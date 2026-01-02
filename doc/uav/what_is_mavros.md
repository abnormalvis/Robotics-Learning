# MAVROS详解 - ROS与MAVLink的桥梁

## 1. MAVROS简介

### 1.1 什么是MAVROS？

**MAVROS** (MAV + ROS) 是一个ROS功能包，它在ROS和运行MAVLink协议的飞控系统之间提供通信桥梁。MAVROS允许ROS节点与PX4、ArduPilot等飞控系统进行交互。

### 1.2 核心功能

- **协议转换**：将MAVLink消息转换为ROS消息（topic/service/action）
- **标准化接口**：提供统一的ROS接口访问飞控功能
- **插件架构**：模块化设计，可扩展支持新功能
- **多平台支持**：兼容PX4、ArduPilot等主流飞控

### 1.3 架构图

```
┌─────────────────────────────────────────────────────┐
│                   ROS应用层                          │
│  (Python/C++ Nodes, 路径规划, 视觉算法等)             │
└────────────────────┬────────────────────────────────┘
                     │ ROS Topics/Services
                     ▼
┌─────────────────────────────────────────────────────┐
│                   MAVROS                             │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐          │
│  │ Plugin 1 │  │ Plugin 2 │  │ Plugin N │          │
│  └──────────┘  └──────────┘  └──────────┘          │
│          MAVLink消息解析与封装                       │
└────────────────────┬────────────────────────────────┘
                     │ MAVLink Protocol
                     ▼
┌─────────────────────────────────────────────────────┐
│              飞控系统 (PX4/ArduPilot)                 │
│  传感器融合、飞行控制、导航等                         │
└─────────────────────────────────────────────────────┘
```

## 2. MAVROS安装与配置

### 2.1 安装MAVROS

#### Ubuntu 20.04 + ROS Noetic
```bash
# 安装MAVROS包
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras

# 安装GeographicLib数据集（用于GPS转换）
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

#### Ubuntu 18.04 + ROS Melodic
```bash
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh
```

#### 从源码编译
```bash
cd ~/catkin_ws/src
git clone https://github.com/mavlink/mavros.git
git clone https://github.com/mavlink/mavlink-gbp-release.git

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build  # 或 catkin_make
```

### 2.2 连接配置

#### 通过USB串口连接
```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600
```

#### 通过UDP连接（常用于SITL仿真）
```bash
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

#### 通过TCP连接
```bash
roslaunch mavros px4.launch fcu_url:=tcp://192.168.1.100:5760
```

### 2.3 Launch文件配置

创建自定义launch文件 `my_mavros.launch`：

```xml
<launch>
    <!-- MAVROS节点 -->
    <node pkg="mavros" type="mavros_node" name="mavros" output="screen">
        <!-- 飞控连接URL -->
        <param name="fcu_url" value="udp://:14540@127.0.0.1:14557"/>
        
        <!-- 地面站连接URL (可选) -->
        <param name="gcs_url" value="udp://@192.168.1.100:14550"/>
        
        <!-- 系统ID和组件ID -->
        <param name="target_system_id" value="1"/>
        <param name="target_component_id" value="1"/>
        
        <!-- 插件黑名单 (禁用不需要的插件以提高性能) -->
        <rosparam command="load" file="$(find mavros)/launch/px4_blacklist.yaml"/>
        
        <!-- 插件配置 -->
        <rosparam command="load" file="$(find mavros)/launch/px4_config.yaml"/>
    </node>
</launch>
```

## 3. MAVROS核心话题(Topics)

### 3.1 状态信息

#### `/mavros/state` - 系统状态
```yaml
类型: mavros_msgs/State
频率: ~1 Hz
内容:
  header: 时间戳
  connected: bool      # 是否连接
  armed: bool          # 是否解锁
  guided: bool         # 是否在GUIDED模式
  manual_input: bool   # 是否有手动输入
  mode: string         # 当前飞行模式
  system_status: uint8 # 系统状态
```

示例订阅：
```python
#!/usr/bin/env python
import rospy
from mavros_msgs.msg import State

def state_callback(msg):
    rospy.loginfo("Connected: %s, Armed: %s, Mode: %s", 
                  msg.connected, msg.armed, msg.mode)

rospy.init_node('state_listener')
rospy.Subscriber('/mavros/state', State, state_callback)
rospy.spin()
```

#### `/mavros/battery` - 电池状态
```yaml
类型: sensor_msgs/BatteryState
内容:
  voltage: float32        # 电压 (V)
  current: float32        # 电流 (A)
  percentage: float32     # 剩余电量百分比
  capacity: float32       # 电池容量 (Ah)
```

### 3.2 位置和姿态

#### `/mavros/local_position/pose` - 本地位置（NED）
```yaml
类型: geometry_msgs/PoseStamped
频率: ~30 Hz
内容:
  header:
    stamp: 时间戳
    frame_id: "map"
  pose:
    position:
      x, y, z: float64  # 位置 (m, NED坐标系)
    orientation:
      x, y, z, w: float64  # 姿态四元数
```

```cpp
// C++订阅示例
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    ROS_INFO("Position: [%.2f, %.2f, %.2f]", 
             msg->pose.position.x,
             msg->pose.position.y,
             msg->pose.position.z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/mavros/local_position/pose", 
                                       10, pose_callback);
    ros::spin();
    return 0;
}
```

#### `/mavros/global_position/global` - 全球位置（GPS）
```yaml
类型: sensor_msgs/NavSatFix
内容:
  latitude: float64      # 纬度 (度)
  longitude: float64     # 经度 (度)
  altitude: float64      # 海拔高度 (m)
```

#### `/mavros/imu/data` - IMU数据
```yaml
类型: sensor_msgs/Imu
频率: ~50 Hz
内容:
  orientation: 四元数姿态
  angular_velocity: 角速度 (rad/s)
  linear_acceleration: 线加速度 (m/s^2)
```

### 3.3 速度信息

#### `/mavros/local_position/velocity_local` - 本地速度
```yaml
类型: geometry_msgs/TwistStamped
内容:
  twist:
    linear: {x, y, z}    # 线速度 (m/s)
    angular: {x, y, z}   # 角速度 (rad/s)
```

### 3.4 控制目标设定

#### `/mavros/setpoint_position/local` - 设置位置目标
```python
from geometry_msgs.msg import PoseStamped

# 创建发布者
setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', 
                               PoseStamped, queue_size=10)

# 发送目标位置
pose = PoseStamped()
pose.header.stamp = rospy.Time.now()
pose.header.frame_id = "map"
pose.pose.position.x = 0.0
pose.pose.position.y = 0.0
pose.pose.position.z = 10.0  # 10米高度

# 需要以至少2Hz的频率持续发送
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    setpoint_pub.publish(pose)
    rate.sleep()
```

#### `/mavros/setpoint_velocity/cmd_vel_unstamped` - 设置速度目标
```python
from geometry_msgs.msg import Twist

vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',
                          Twist, queue_size=10)

twist = Twist()
twist.linear.x = 1.0   # 前进1 m/s
twist.linear.y = 0.0
twist.linear.z = 0.0
twist.angular.z = 0.5  # 旋转0.5 rad/s

vel_pub.publish(twist)
```

#### `/mavros/setpoint_raw/local` - 原始设置点（更灵活）
```python
from mavros_msgs.msg import PositionTarget

setpoint_raw_pub = rospy.Publisher('/mavros/setpoint_raw/local',
                                   PositionTarget, queue_size=10)

target = PositionTarget()
target.header.stamp = rospy.Time.now()
target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

# 设置控制模式位掩码
# 位掩码说明：
# 0b0000111111111000 = 0x0F80 = 忽略位置
# 0b0000111111000111 = 0x0FC7 = 忽略速度
# 0b0000111000111111 = 0x0E3F = 忽略加速度
target.type_mask = PositionTarget.IGNORE_VX | \
                   PositionTarget.IGNORE_VY | \
                   PositionTarget.IGNORE_VZ | \
                   PositionTarget.IGNORE_AFX | \
                   PositionTarget.IGNORE_AFY | \
                   PositionTarget.IGNORE_AFZ | \
                   PositionTarget.IGNORE_YAW_RATE

# 设置位置
target.position.x = 5.0
target.position.y = 5.0
target.position.z = 10.0
target.yaw = 1.57  # 朝向东方

setpoint_raw_pub.publish(target)
```

## 4. MAVROS核心服务(Services)

### 4.1 模式切换

#### `/mavros/set_mode` - 设置飞行模式
```python
from mavros_msgs.srv import SetMode
import rospy

rospy.wait_for_service('/mavros/set_mode')
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

# PX4模式
response = set_mode(custom_mode="OFFBOARD")

# ArduCopter模式
# response = set_mode(custom_mode="GUIDED")

if response.mode_sent:
    rospy.loginfo("模式切换成功")
```

常用飞行模式：
- **PX4**: MANUAL, STABILIZED, ACRO, RATTITUDE, ALTCTL, POSCTL, OFFBOARD, AUTO
- **ArduCopter**: STABILIZE, ALT_HOLD, LOITER, GUIDED, AUTO, RTL, LAND

### 4.2 解锁/上锁

#### `/mavros/cmd/arming` - 电机解锁
```python
from mavros_msgs.srv import CommandBool

rospy.wait_for_service('/mavros/cmd/arming')
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

# 解锁
response = arming_client(True)
if response.success:
    rospy.loginfo("解锁成功")

# 上锁
# response = arming_client(False)
```

### 4.3 起飞和降落

#### `/mavros/cmd/takeoff` - 起飞
```python
from mavros_msgs.srv import CommandTOL

rospy.wait_for_service('/mavros/cmd/takeoff')
takeoff_client = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

# 起飞到10米
response = takeoff_client(
    altitude=10.0,
    latitude=0,    # 使用当前位置
    longitude=0,
    min_pitch=0,
    yaw=0
)
```

#### `/mavros/cmd/land` - 降落
```python
from mavros_msgs.srv import CommandTOL

rospy.wait_for_service('/mavros/cmd/land')
land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

response = land_client(
    altitude=0,
    latitude=0,
    longitude=0,
    min_pitch=0,
    yaw=0
)
```

### 4.4 航点任务

#### `/mavros/mission/push` - 上传航点
```python
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint

rospy.wait_for_service('/mavros/mission/push')
waypoint_push = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)

# 创建航点列表
waypoints = []

# 起飞点
wp0 = Waypoint()
wp0.frame = Waypoint.FRAME_GLOBAL_REL_ALT
wp0.command = 22  # MAV_CMD_NAV_TAKEOFF
wp0.is_current = True
wp0.autocontinue = True
wp0.param1 = 0  # pitch
wp0.z_alt = 10  # 起飞高度
waypoints.append(wp0)

# 导航点1
wp1 = Waypoint()
wp1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
wp1.command = 16  # MAV_CMD_NAV_WAYPOINT
wp1.is_current = False
wp1.autocontinue = True
wp1.x_lat = 47.398  # 纬度
wp1.y_long = 8.545  # 经度
wp1.z_alt = 10
waypoints.append(wp1)

# 降落点
wp2 = Waypoint()
wp2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
wp2.command = 21  # MAV_CMD_NAV_LAND
wp2.autocontinue = True
waypoints.append(wp2)

# 上传
response = waypoint_push(waypoints=waypoints)
if response.success:
    rospy.loginfo("航点上传成功")
```

### 4.5 坐标系设置

#### `/mavros/setpoint_position/mav_frame` - 设置坐标系
```python
rospy.set_param('/mavros/setpoint_position/mav_frame', 'FRAME_LOCAL_NED')
```

## 5. MAVROS插件系统

### 5.1 常用插件

| 插件名 | 功能 | 主要话题/服务 |
|--------|------|--------------|
| **sys_status** | 系统状态 | `/mavros/state`, `/mavros/battery` |
| **imu** | IMU数据 | `/mavros/imu/data`, `/mavros/imu/data_raw` |
| **local_position** | 本地位置估计 | `/mavros/local_position/pose` |
| **global_position** | GPS位置 | `/mavros/global_position/global` |
| **setpoint_position** | 位置控制 | `/mavros/setpoint_position/local` |
| **setpoint_velocity** | 速度控制 | `/mavros/setpoint_velocity/cmd_vel` |
| **command** | 命令接口 | `/mavros/cmd/arming`, `/mavros/cmd/takeoff` |
| **waypoint** | 航点任务 | `/mavros/mission/waypoints` |
| **rc_io** | 遥控器输入 | `/mavros/rc/in`, `/mavros/rc/out` |
| **param** | 参数管理 | `/mavros/param/get`, `/mavros/param/set` |

### 5.2 插件配置文件

编辑 `px4_config.yaml` 配置插件参数：

```yaml
# 连接配置
conn:
  heartbeat_rate: 1.0    # 心跳频率 (Hz)
  timeout: 10.0          # 超时时间 (秒)
  timesync_rate: 10.0    # 时间同步频率
  system_time_rate: 1.0  # 系统时间发布频率

# 系统插件
sys:
  min_voltage: 10.0      # 最小电压报警
  disable_diag: false    # 禁用诊断

# 时间同步
time:
  time_ref_source: "fcu"  # 时间参考源
  timesync_mode: MAVLINK
  timesync_avg_alpha: 0.6

# 本地位置
local_position:
  frame_id: "map"
  tf:
    send: true            # 发送TF变换
    frame_id: "map"
    child_frame_id: "base_link"
    rate_limit: 10.0

# IMU
imu:
  frame_id: "base_link"
  linear_acceleration_stdev: 0.0003
  angular_velocity_stdev: 0.0003
  orientation_stdev: 1.0
  magnetic_stdev: 0.0

# 设置点插件
setpoint_position:
  tf:
    listen: false
    frame_id: "map"
    child_frame_id: "target_position"
    rate_limit: 50.0
  mav_frame: FRAME_LOCAL_NED

# RC通道映射
rc:
  channels: 18
```

## 6. MAVROS完整飞行示例

### 6.1 基础飞行脚本

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardControl:
    def __init__(self):
        rospy.init_node('offboard_control_node')
        
        # 订阅状态
        self.current_state = State()
        self.state_sub = rospy.Subscriber('/mavros/state', State, 
                                          self.state_callback)
        
        # 发布本地位置
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local',
                                             PoseStamped, queue_size=10)
        
        # 等待服务
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', 
                                                CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', 
                                                  SetMode)
        
        # 设置发布频率
        self.rate = rospy.Rate(20)
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def wait_for_connect(self):
        """等待连接到飞控"""
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("等待连接...")
            self.rate.sleep()
        rospy.loginfo("已连接到飞控")
        
    def send_setpoint_before_offboard(self):
        """在进入OFFBOARD模式前发送设置点"""
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2
        
        # 发送100个点（5秒）
        for i in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(pose)
            self.rate.sleep()
            
    def set_offboard_mode(self):
        """切换到OFFBOARD模式"""
        if self.current_state.mode != "OFFBOARD":
            response = self.set_mode_client(custom_mode='OFFBOARD')
            if response.mode_sent:
                rospy.loginfo("OFFBOARD模式已启用")
                return True
        return False
        
    def arm(self):
        """解锁"""
        if not self.current_state.armed:
            response = self.arming_client(True)
            if response.success:
                rospy.loginfo("无人机已解锁")
                return True
        return False
        
    def fly_to_position(self, x, y, z, duration=5.0):
        """飞到指定位置"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        rospy.loginfo("飞向位置: (%.2f, %.2f, %.2f)" % (x, y, z))
        
        timeout = rospy.Time.now() + rospy.Duration(duration)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(pose)
            self.rate.sleep()
            
    def run(self):
        """主飞行流程"""
        # 1. 等待连接
        self.wait_for_connect()
        
        # 2. 发送初始设置点
        rospy.loginfo("发送初始设置点...")
        self.send_setpoint_before_offboard()
        
        # 3. 切换到OFFBOARD模式
        self.set_offboard_mode()
        
        # 4. 解锁
        self.arm()
        
        # 5. 执行飞行任务
        rospy.loginfo("开始飞行任务...")
        
        # 起飞到2米
        self.fly_to_position(0, 0, 2, duration=5)
        rospy.sleep(2)
        
        # 飞到点1
        self.fly_to_position(5, 0, 2, duration=10)
        rospy.sleep(2)
        
        # 飞到点2
        self.fly_to_position(5, 5, 2, duration=10)
        rospy.sleep(2)
        
        # 飞到点3
        self.fly_to_position(0, 5, 2, duration=10)
        rospy.sleep(2)
        
        # 返回起点
        self.fly_to_position(0, 0, 2, duration=10)
        rospy.sleep(2)
        
        # 降落
        self.fly_to_position(0, 0, 0.5, duration=5)
        
        rospy.loginfo("任务完成")

if __name__ == '__main__':
    try:
        controller = OffboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
```

### 6.2 使用方法

```bash
# 终端1：启动MAVROS（连接到PX4 SITL）
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557

# 终端2：运行飞行脚本
chmod +x offboard_control.py
rosrun your_package offboard_control.py
```

## 7. MAVROS与PX4/Gazebo仿真

### 7.1 启动PX4 SITL仿真

```bash
# 克隆PX4源码
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

# 编译并启动SITL
make px4_sitl gazebo
```

### 7.2 完整仿真launch文件

```xml
<launch>
    <!-- 启动PX4 SITL -->
    <include file="$(find px4)/launch/posix_sitl.launch">
        <arg name="vehicle" value="iris"/>
    </include>
    
    <!-- 启动MAVROS -->
    <include file="$(find mavros)/launch/px4.launch">
        <arg name="fcu_url" value="udp://:14540@127.0.0.1:14557"/>
        <arg name="gcs_url" value=""/>
        <arg name="tgt_system" value="1"/>
        <arg name="tgt_component" value="1"/>
    </include>
    
    <!-- 启动你的控制节点 -->
    <node pkg="your_package" type="offboard_control.py" 
          name="offboard_control" output="screen"/>
</launch>
```

## 8. 坐标系变换(TF)

MAVROS自动发布以下TF变换：

```
map -> odom -> base_link -> base_link_frd
                           -> camera_link
                           -> lidar_link
```

### 8.1 查看TF树
```bash
rosrun rqt_tf_tree rqt_tf_tree
# 或
rosrun tf view_frames
evince frames.pdf
```

### 8.2 坐标系说明

- **map**: 世界坐标系（固定参考系）
- **odom**: 里程计坐标系
- **base_link**: 机体坐标系（FLU - Forward, Left, Up）
- **base_link_frd**: 机体坐标系（FRD - Forward, Right, Down，符合航空惯例）

## 9. 参数管理

### 9.1 读取参数
```python
from mavros_msgs.srv import ParamGet

rospy.wait_for_service('/mavros/param/get')
param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)

# 获取参数
response = param_get(param_id='MPC_Z_VEL_MAX_UP')
rospy.loginfo("参数值: %.2f" % response.value.real)
```

### 9.2 设置参数
```python
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue

rospy.wait_for_service('/mavros/param/set')
param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)

# 设置参数
param_value = ParamValue()
param_value.integer = 0
param_value.real = 5.0

response = param_set(param_id='MPC_Z_VEL_MAX_UP', value=param_value)
if response.success:
    rospy.loginfo("参数设置成功")
```

### 9.3 批量获取参数
```bash
# 获取所有参数
rosrun mavros mavparam dump /tmp/params.txt

# 加载参数
rosrun mavros mavparam load /tmp/params.txt
```

## 10. 故障排查

### 10.1 常见问题

#### Q1: MAVROS无法连接
```bash
# 检查端口
ls -l /dev/ttyACM*

# 检查权限
sudo usermod -a -G dialout $USER
# 注销后重新登录

# 检查MAVLink连接
rostopic echo /mavros/state
```

#### Q2: 无法切换到OFFBOARD模式
- 确保持续发送设置点（≥2Hz）
- 检查是否已连接RC或在仿真中禁用RC检查
- 查看 `/mavros/state` 确认连接状态

```python
# 持续发送设置点
while not rospy.is_shutdown():
    local_pos_pub.publish(pose)
    rate.sleep()  # 20Hz
```

#### Q3: 无法解锁
```bash
# 检查安全开关（真实飞机）
# 检查EKF状态
rostopic echo /mavros/extended_state

# 查看错误信息
rostopic echo /mavros/statustext/recv
```

### 10.2 调试工具

```bash
# 查看所有话题
rostopic list

# 监控特定话题
rostopic echo /mavros/local_position/pose

# 查看消息类型
rostopic type /mavros/state
rosmsg show mavros_msgs/State

# 查看服务
rosservice list
rosservice info /mavros/set_mode

# 使用mavproxy监控
mavproxy.py --master=/dev/ttyACM0 --baudrate=921600
```

### 10.3 性能优化

```yaml
# 在配置文件中禁用不需要的插件
plugin_blacklist:
  - image_pub
  - vision_speed_estimate
  - mocap_pose_estimate
  - fake_gps
  - camera

# 降低某些消息的频率
conn:
  heartbeat_rate: 1.0
  timesync_rate: 5.0
```

## 11. 高级功能

### 11.1 视觉定位(VIO)
```python
from geometry_msgs.msg import PoseStamped

# 发布视觉位置估计
vision_pub = rospy.Publisher('/mavros/vision_pose/pose',
                            PoseStamped, queue_size=10)

pose = PoseStamped()
pose.header.stamp = rospy.Time.now()
pose.header.frame_id = "odom"
pose.pose.position.x = x  # 从SLAM/VIO获取
pose.pose.position.y = y
pose.pose.position.z = z
pose.pose.orientation = quaternion

vision_pub.publish(pose)
```

### 11.2 避障
```python
# 发布障碍物距离
from sensor_msgs.msg import Range

distance_pub = rospy.Publisher('/mavros/distance_sensor/rangefinder',
                              Range, queue_size=10)
```

### 11.3 自定义MAVLink消息
```bash
# 在mavros中添加自定义插件
# 1. 定义MAVLink消息（XML）
# 2. 生成代码
# 3. 创建MAVROS插件
# 4. 重新编译
```

## 12. 最佳实践

### 12.1 安全检查清单
- ✅ 始终在仿真中测试
- ✅ 实现急停机制
- ✅ 监控电池电量
- ✅ 设置地理围栏
- ✅ 实现失控保护(Failsafe)
- ✅ 检查GPS信号质量

### 12.2 代码规范
```python
# 良好的错误处理
try:
    response = arming_client(True)
    if not response.success:
        rospy.logerr("解锁失败")
        return
except rospy.ServiceException as e:
    rospy.logerr("服务调用失败: %s" % e)
    return

# 超时保护
timeout = rospy.Time.now() + rospy.Duration(10)
while rospy.Time.now() < timeout:
    if condition_met:
        break
    rate.sleep()
```

### 12.3 性能建议
- 使用适当的发布频率（位置控制20Hz足够）
- 避免阻塞回调函数
- 使用多线程处理计算密集型任务

## 13. 学习资源

### 13.1 官方文档
- [MAVROS Wiki](http://wiki.ros.org/mavros)
- [MAVROS GitHub](https://github.com/mavlink/mavros)
- [PX4开发指南](https://docs.px4.io/)
- [MAVLink协议](https://mavlink.io/)

### 13.2 示例项目
- [MAVROS Offboard示例](https://github.com/mavlink/mavros/tree/master/mavros_extras/src/plugins)
- [PX4示例代码](https://github.com/PX4/PX4-Autopilot/tree/master/src/examples)

### 13.3 相关包
- **mavros_extras**: 额外的MAVROS插件
- **mavros_msgs**: MAVROS消息定义
- **geographic_msgs**: 地理坐标消息

## 14. 与MAVLink对比总结

| 特性 | MAVLink | MAVROS |
|------|---------|--------|
| **层次** | 底层通信协议 | ROS包装层 |
| **使用场景** | 直接与飞控通信 | ROS系统集成 |
| **编程接口** | C/C++/Python库 | ROS Topics/Services |
| **学习曲线** | 需要了解协议细节 | 使用ROS接口即可 |
| **灵活性** | 完全控制 | 抽象程度高 |
| **适用对象** | 固件开发者 | 应用开发者 |

---

**最后更新**: 2026年1月2日

**相关文档**: [MAVLink协议详解](mavlink_protocal.md)
