# FASTLIO2 ROS2
## 主要工作
1. 重构[FASTLIO2](https://github.com/hku-mars/FAST_LIO) 适配ROS2
2. 添加回环节点，基于位置先验+ICP进行回环检测，基于GTSAM进行位姿图优化
3. 添加重定位节点，基于由粗到细两阶段ICP进行重定位
4. 增加一致性地图优化，基于[BLAM](https://github.com/hku-mars/BALM) (小场景地图) 和[HBA](https://github.com/hku-mars/HBA) (大场景地图)

## 环境依赖
1. Ubuntu 22.04
2. ROS2 Humble

## 编译依赖
```text
pcl
Eigen
sophus
gtsam
```

> 注: fastlio2 已移除 livox_ros_driver2 依赖，通过标准 `sensor_msgs/msg/PointCloud2`
> 接收双雷达融合节点发布的 `/merged_points`（需包含单点时间字段 `time` 或 `timestamp`）。

## 详细说明
### 1.编译 Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

**新的Sophus依赖fmt，可以在CMakeLists.txt中添加add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)去除，否则会报错**


## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb 
--来自百度网盘超级会员v7的分享
```

## 部分脚本

### 1.激光惯性里程计 
```shell
ros2 launch fastlio2 lio_launch.py
ros2 bag play your_bag_file
```

### 2.里程计加回环
#### 启动回环节点
```shell
ros2 launch pgo pgo_launch.py
ros2 bag play your_bag_file
```
#### 保存地图
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3.里程计加重定位
#### 启动重定位节点
```shell
ros2 launch localizer localizer_launch.py
ros2 bag play your_bag_file // 可选
```
#### 设置重定位初始值
```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```
#### 检查重定位结果
```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 4.一致性地图优化
#### 启动一致性地图优化节点
```shell
ros2 launch hba hba_launch.py
```
#### 调用优化服务
```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```
**如果需要调用优化服务，保存地图时需要设置save_patches为true**

## 特别感谢
1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [BLAM](https://github.com/hku-mars/BALM)
3. [HBA](https://github.com/hku-mars/HBA)
## 性能相关的问题
该代码主要使用timerCB作为频率触发主函数。早期版本中 ROS2 回调运行在单线程执行器上，电脑性能不好时会出现调用阻塞的情况；现已改为 **MultiThreadedExecutor**，并为 IMU 回调、雷达回调、定时器回调分配独立的回调组，三者可并行执行，建议在低算力工控机上保持该配置。若需进一步限制线程数，可在 `lio_node.cpp` 中构造 `MultiThreadedExecutor` 时显式传入线程数。

