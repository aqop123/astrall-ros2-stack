#include "interface.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

geometry_msgs::msg::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

AstrallSubscribeFreq subscription_frequency(int hz) {
  switch (hz) {
    case 1: return ASTRALL_SUB_FREQ_1HZ;
    case 25: return ASTRALL_SUB_FREQ_25HZ;
    case 50: return ASTRALL_SUB_FREQ_50HZ;
    case 125: return ASTRALL_SUB_FREQ_125HZ;
    case 250: return ASTRALL_SUB_FREQ_250HZ;
    default: throw std::invalid_argument("imu_frequency_hz must be 1, 25, 50, 125, or 250");
  }
}

}  // namespace

class AstrallSensorNode : public rclcpp::Node {
 public:
  AstrallSensorNode() : Node("astrall_sensor_node") {
    const int frequency = declare_parameter<int>("imu_frequency_hz", 50);
    const int init_timeout = declare_parameter<int>("sdk_init_timeout_ms", 60000);
    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
    camera_frame_ = declare_parameter<std::string>("camera_frame_id", "camera_link");
    camera_enabled_ = declare_parameter<bool>("camera_enabled", true);
    camera_port_ = declare_parameter<int>("camera_udp_port", 6000);
    camera_source_ip_ = declare_parameter<std::string>("camera_source_ip", "");
    odom_xy_variance_ = declare_parameter<double>("odom_xy_variance", 100.0);
    odom_yaw_variance_ = declare_parameter<double>("odom_yaw_variance", 100.0);
    const auto sdk_frequency = subscription_frequency(frequency);
    if (init_timeout <= 0 || camera_port_ <= 0 || camera_port_ > 65535 ||
        odom_xy_variance_ < 0 || odom_yaw_variance_ < 0) {
      throw std::invalid_argument("Invalid timeout, camera port, or odometry variance");
    }

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::SensorDataQoS());
    h264_packet_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "camera/h264_packet", rclcpp::SensorDataQoS());

    if (camera_enabled_) open_camera_socket();

    sdk_config_.heartbeatCb = [](void *, uint16_t) {};
    sdk_config_.sdkStatusCb = [this](void *raw, uint16_t length) {
      if (!raw || length < sizeof(AstrallSdkStatus)) return;
      const auto *status = static_cast<const AstrallSdkStatus *>(raw);
      linked_.store(status->link != 0);
    };

    RCLCPP_INFO(get_logger(), "Connecting to Astrall SDK (timeout %d ms)...", init_timeout);
    const auto init_result = AstrallSdkInit(sdk_config_, static_cast<uint32_t>(init_timeout));
    if (init_result != ASTRALL_RES_SUCCESSED) {
      if (camera_socket_ >= 0) close(camera_socket_);
      camera_socket_ = -1;
      throw std::runtime_error("AstrallSdkInit failed: " + std::to_string(init_result));
    }
    sdk_initialized_ = true;
    linked_.store(true);

    const auto imu_result = AstrallSubscriptionData(
      ASTRALL_SUB_TOPIC_ID_IMU, sdk_frequency,
      [this](void *raw, uint16_t length) { on_imu(raw, length); });
    if (imu_result != ASTRALL_RES_SUCCESSED) {
      RCLCPP_ERROR(get_logger(), "IMU subscription failed: 0x%04X", imu_result);
      cleanup();
      throw std::runtime_error("IMU subscription failed");
    }

    if (camera_enabled_) {
      const auto camera_result = AstrallSubscriptionData(
        ASTRALL_SUB_TOPIC_ID_CAMERA_RGB, ASTRALL_SUB_FREQ_1HZ,
        [](void *, uint16_t) {});
      if (camera_result != ASTRALL_RES_SUCCESSED) {
        RCLCPP_WARN(get_logger(), "Camera subscription failed: 0x%04X; IMU/odom remain active",
                    camera_result);
        close(camera_socket_);
        camera_socket_ = -1;
      } else {
        camera_running_.store(true);
        camera_thread_ = std::thread([this] { camera_loop(); });
      }
    }

    heartbeat_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
      const auto result = AstrallHeartbeat();
      if (result != ASTRALL_RES_SUCCESSED) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Astrall heartbeat failed: 0x%04X", result);
      }
    });
    RCLCPP_INFO(get_logger(), "Sensor bridge ready; no motor control is requested");
  }

  ~AstrallSensorNode() override { cleanup(); }

 private:
  void cleanup() {
    if (heartbeat_timer_) heartbeat_timer_->cancel();
    camera_running_.store(false);
    if (camera_thread_.joinable()) camera_thread_.join();
    if (camera_socket_ >= 0) {
      close(camera_socket_);
      camera_socket_ = -1;
    }
    if (sdk_initialized_) {
      AstrallSdkDeinit();
      sdk_initialized_ = false;
    }
  }

  void on_imu(void *raw, uint16_t length) {
    if (!raw || length != sizeof(AstrallImuData)) return;
    const auto *data = static_cast<const AstrallImuData *>(raw);
    if (!std::isfinite(data->odomX) || !std::isfinite(data->odomY) ||
        !std::isfinite(data->roll) || !std::isfinite(data->pitch) ||
        !std::isfinite(data->yaw)) return;

    // SDK timestamp units and epoch are undocumented; stamp with ROS receive time.
    const auto stamp = now();
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = imu_frame_;
    imu.orientation = quaternion_from_rpy(data->roll, data->pitch, data->yaw);
    imu.angular_velocity.x = data->gyroscope[0];
    imu.angular_velocity.y = data->gyroscope[1];
    imu.angular_velocity.z = data->gyroscope[2];
    imu.linear_acceleration.x = data->accelerometer[0];
    imu.linear_acceleration.y = data->accelerometer[1];
    imu.linear_acceleration.z = data->accelerometer[2];
    // Zero covariance denotes unknown precision, not zero measurement error.
    imu_pub_->publish(imu);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = data->odomX;
    odom.pose.pose.position.y = data->odomY;
    odom.pose.pose.orientation = quaternion_from_rpy(0.0, 0.0, data->yaw);
    odom.pose.covariance[0] = odom_xy_variance_;
    odom.pose.covariance[7] = odom_xy_variance_;
    odom.pose.covariance[14] = 1e6;  // z is not supplied by the SDK.
    odom.pose.covariance[21] = 1e6;  // roll is not planar odometry.
    odom.pose.covariance[28] = 1e6;  // pitch is not planar odometry.
    odom.pose.covariance[35] = odom_yaw_variance_;
    for (size_t i : {0U, 7U, 14U, 21U, 28U, 35U}) odom.twist.covariance[i] = 1e6;
    odom_pub_->publish(odom);
  }

  void open_camera_socket() {
    camera_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (camera_socket_ < 0) throw std::runtime_error("Cannot open camera UDP socket");
    timeval timeout{0, 200000};
    setsockopt(camera_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(camera_port_));
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(camera_socket_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      const std::string error = std::strerror(errno);
      close(camera_socket_);
      camera_socket_ = -1;
      throw std::runtime_error("Cannot bind camera UDP port: " + error);
    }
    if (!camera_source_ip_.empty()) {
      in_addr source{};
      if (inet_pton(AF_INET, camera_source_ip_.c_str(), &source) != 1) {
        close(camera_socket_);
        camera_socket_ = -1;
        throw std::invalid_argument("camera_source_ip must be an IPv4 address");
      }
      camera_source_address_ = source.s_addr;
    }
    RCLCPP_INFO(get_logger(), "Listening for H.264 camera UDP data on port %d", camera_port_);
  }

  void camera_loop() {
    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    AVCodecParserContext *parser = av_parser_init(AV_CODEC_ID_H264);
    AVCodecContext *decoder = codec ? avcodec_alloc_context3(codec) : nullptr;
    AVFrame *frame = av_frame_alloc();
    AVPacket *packet = av_packet_alloc();
    SwsContext *converter = nullptr;
    if (!codec || !parser || !decoder || !frame || !packet || avcodec_open2(decoder, codec, nullptr) < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot initialize FFmpeg H.264 decoder");
      camera_running_.store(false);
    }
    std::array<uint8_t, 65536 + AV_INPUT_BUFFER_PADDING_SIZE> buffer{};
    bool reported_packets = false;
    bool reported_frames = false;
    while (camera_running_.load() && rclcpp::ok()) {
      sockaddr_in source{};
      socklen_t source_length = sizeof(source);
      const auto size = recvfrom(camera_socket_, buffer.data(), 65536, 0,
                                 reinterpret_cast<sockaddr *>(&source), &source_length);
      if (size <= 0) {
        if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Camera UDP receive error: %s", std::strerror(errno));
        }
        continue;
      }
      if (camera_source_address_ && source.sin_addr.s_addr != camera_source_address_) continue;
      sensor_msgs::msg::CompressedImage raw_packet;
      raw_packet.header.stamp = now();
      raw_packet.header.frame_id = camera_frame_;
      raw_packet.format = "h264/udp-packet";
      raw_packet.data.assign(buffer.begin(), buffer.begin() + size);
      h264_packet_pub_->publish(std::move(raw_packet));
      if (!reported_packets) {
        RCLCPP_INFO(get_logger(), "Receiving camera UDP packets");
        reported_packets = true;
      }
      int remaining = static_cast<int>(size);
      uint8_t *cursor = buffer.data();
      int zero_consumed_outputs = 0;
      while (remaining > 0) {
        uint8_t *parsed = nullptr;
        int parsed_size = 0;
        const int consumed = av_parser_parse2(parser, decoder, &parsed, &parsed_size,
                                              cursor, remaining, AV_NOPTS_VALUE,
                                              AV_NOPTS_VALUE, 0);
        if (consumed < 0) break;
        cursor += consumed;
        remaining -= consumed;
        if (parsed_size > 0) {
          packet->data = parsed;
          packet->size = parsed_size;
          if (avcodec_send_packet(decoder, packet) == 0) {
            while (avcodec_receive_frame(decoder, frame) == 0) {
              converter = sws_getCachedContext(converter, frame->width, frame->height,
                                               static_cast<AVPixelFormat>(frame->format),
                                               frame->width, frame->height, AV_PIX_FMT_RGB24,
                                               SWS_BILINEAR, nullptr, nullptr, nullptr);
              if (!converter) break;
              sensor_msgs::msg::Image image;
              image.header.stamp = now();
              image.header.frame_id = camera_frame_;
              image.width = static_cast<uint32_t>(frame->width);
              image.height = static_cast<uint32_t>(frame->height);
              image.encoding = "rgb8";
              image.step = image.width * 3;
              image.data.resize(static_cast<size_t>(image.step) * image.height);
              uint8_t *out_data[4] = {image.data.data(), nullptr, nullptr, nullptr};
              int out_linesize[4] = {static_cast<int>(image.step), 0, 0, 0};
              sws_scale(converter, frame->data, frame->linesize, 0, frame->height,
                        out_data, out_linesize);
              image_pub_->publish(std::move(image));
              if (!reported_frames) {
                RCLCPP_INFO(get_logger(), "Publishing decoded camera images");
                reported_frames = true;
              }
            }
          }
        }
        if (consumed == 0) {
          // A parser can emit the previous frame before consuming new input.
          if (parsed_size == 0 || ++zero_consumed_outputs > 2) break;
        } else {
          zero_consumed_outputs = 0;
        }
      }
    }
    sws_freeContext(converter);
    av_packet_free(&packet);
    av_frame_free(&frame);
    avcodec_free_context(&decoder);
    if (parser) av_parser_close(parser);
  }

  std::string imu_frame_, odom_frame_, base_frame_, camera_frame_, camera_source_ip_;
  AstrallConfig sdk_config_;
  int camera_port_ = 6000;
  bool camera_enabled_ = true;
  bool sdk_initialized_ = false;
  double odom_xy_variance_ = 100.0, odom_yaw_variance_ = 100.0;
  int camera_socket_ = -1;
  uint32_t camera_source_address_ = 0;
  std::atomic<bool> camera_running_{false};
  std::atomic<bool> linked_{false};
  std::thread camera_thread_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr h264_packet_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<AstrallSensorNode>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("astrall_sensor_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
