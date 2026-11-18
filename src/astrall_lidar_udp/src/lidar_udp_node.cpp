#include <astrall_lidar_udp/msg/udp_packet.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Channel {
  std::string label;
  std::string expected_ip;
  std::string frame_id;
  uint16_t port = 0;
  int socket_fd = -1;
  rclcpp::Publisher<astrall_lidar_udp::msg::UdpPacket>::SharedPtr publisher;
  std::atomic<uint64_t> received{0};
  std::atomic<uint64_t> filtered{0};
  bool logged_first_packet = false;
  bool logged_wrong_source = false;

  ~Channel() {
    if (socket_fd >= 0) close(socket_fd);
  }
};

std::string first_bytes(const uint8_t *data, size_t length) {
  std::ostringstream output;
  for (size_t i = 0; i < std::min(length, size_t{16}); ++i) {
    if (i) output << ' ';
    output << std::hex << std::setfill('0') << std::setw(2)
           << static_cast<unsigned>(data[i]);
  }
  return output.str();
}

}  // namespace

class LidarUdpNode : public rclcpp::Node {
 public:
  LidarUdpNode() : Node("lidar_udp_node") {
    const auto bind_address = declare_parameter<std::string>("bind_address", "0.0.0.0");
    const auto front_ip = declare_parameter<std::string>("front_ip", "10.18.0.120");
    const auto rear_ip = declare_parameter<std::string>("rear_ip", "10.18.0.121");
    filter_source_ip_ = declare_parameter<bool>("filter_source_ip", true);

    add_channel("front/msop", front_ip, "front_lidar",
                declare_parameter<int>("front_msop_port", 6699), bind_address);
    add_channel("front/difop", front_ip, "front_lidar",
                declare_parameter<int>("front_difop_port", 7788), bind_address);
    add_channel("front/imu", front_ip, "front_lidar",
                declare_parameter<int>("front_imu_port", 6688), bind_address);
    add_channel("rear/msop", rear_ip, "rear_lidar",
                declare_parameter<int>("rear_msop_port", 6969), bind_address);
    add_channel("rear/difop", rear_ip, "rear_lidar",
                declare_parameter<int>("rear_difop_port", 7878), bind_address);
    add_channel("rear/imu", rear_ip, "rear_lidar",
                declare_parameter<int>("rear_imu_port", 6868), bind_address);

    running_.store(true);
    receiver_ = std::thread([this] { receive_loop(); });
    stats_timer_ = create_wall_timer(std::chrono::seconds(10), [this] {
      for (const auto &channel : channels_) {
        RCLCPP_INFO(get_logger(), "%s UDP %u: %llu received, %llu filtered",
                    channel->label.c_str(), channel->port,
                    static_cast<unsigned long long>(channel->received.load()),
                    static_cast<unsigned long long>(channel->filtered.load()));
      }
    });
  }

  ~LidarUdpNode() override {
    if (stats_timer_) stats_timer_->cancel();
    running_.store(false);
    if (receiver_.joinable()) receiver_.join();
  }

 private:
  void add_channel(const std::string &label, const std::string &source_ip,
                   const std::string &frame_id, int port, const std::string &bind_address) {
    if (port <= 0 || port > 65535) throw std::invalid_argument("Invalid UDP port for " + label);
    auto channel = std::make_unique<Channel>();
    channel->label = label;
    channel->expected_ip = source_ip;
    channel->frame_id = frame_id;
    channel->port = static_cast<uint16_t>(port);
    channel->publisher = create_publisher<astrall_lidar_udp::msg::UdpPacket>(
      "lidar/" + label + "_packets", rclcpp::SensorDataQoS());
    channel->socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (channel->socket_fd < 0) {
      throw std::runtime_error("Cannot create UDP socket for " + label + ": " + std::strerror(errno));
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(channel->port);
    if (inet_pton(AF_INET, bind_address.c_str(), &address.sin_addr) != 1) {
      throw std::invalid_argument("bind_address must be an IPv4 address");
    }
    if (bind(channel->socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      throw std::runtime_error("Cannot bind UDP " + std::to_string(port) + ": " +
                               std::strerror(errno));
    }
    RCLCPP_INFO(get_logger(), "Listening for %s on %s:%d", label.c_str(),
                bind_address.c_str(), port);
    channels_.push_back(std::move(channel));
  }

  void receive_loop() {
    std::array<uint8_t, 65536> buffer{};
    while (running_.load() && rclcpp::ok()) {
      std::vector<pollfd> descriptors;
      descriptors.reserve(channels_.size());
      for (const auto &channel : channels_) descriptors.push_back({channel->socket_fd, POLLIN, 0});
      const int ready = poll(descriptors.data(), descriptors.size(), 100);
      if (ready <= 0) continue;
      for (size_t index = 0; index < channels_.size(); ++index) {
        if (!(descriptors[index].revents & POLLIN)) continue;
        auto &channel = *channels_[index];
        for (int drained = 0; drained < 256; ++drained) {
          sockaddr_in source{};
          socklen_t source_length = sizeof(source);
          const auto length = recvfrom(channel.socket_fd, buffer.data(), buffer.size(), MSG_DONTWAIT,
                                       reinterpret_cast<sockaddr *>(&source), &source_length);
          if (length < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                   "UDP receive error on %u: %s", channel.port, std::strerror(errno));
            }
            break;
          }
          char source_text[INET_ADDRSTRLEN]{};
          inet_ntop(AF_INET, &source.sin_addr, source_text, sizeof(source_text));
          const std::string sender(source_text);
          if (filter_source_ip_ && sender != channel.expected_ip) {
            ++channel.filtered;
            if (!channel.logged_wrong_source) {
              RCLCPP_WARN(get_logger(), "%s received from unexpected IP %s; expected %s",
                          channel.label.c_str(), sender.c_str(), channel.expected_ip.c_str());
              channel.logged_wrong_source = true;
            }
            continue;
          }
          astrall_lidar_udp::msg::UdpPacket packet;
          packet.header.stamp = now();
          packet.header.frame_id = channel.frame_id;
          packet.source_ip = sender;
          packet.source_port = ntohs(source.sin_port);
          packet.destination_port = channel.port;
          packet.data.assign(buffer.begin(), buffer.begin() + length);
          channel.publisher->publish(std::move(packet));
          ++channel.received;
          if (!channel.logged_first_packet) {
            RCLCPP_INFO(get_logger(), "%s first packet: %ld bytes from %s:%u; first bytes: %s",
                        channel.label.c_str(), static_cast<long>(length), sender.c_str(),
                        ntohs(source.sin_port), first_bytes(buffer.data(), length).c_str());
            channel.logged_first_packet = true;
          }
        }
      }
    }
  }

  bool filter_source_ip_ = true;
  std::atomic<bool> running_{false};
  std::vector<std::unique_ptr<Channel>> channels_;
  std::thread receiver_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<LidarUdpNode>();
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("lidar_udp_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
