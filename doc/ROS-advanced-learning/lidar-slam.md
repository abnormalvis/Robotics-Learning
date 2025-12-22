作者：喻晨辉

## ROS激光雷达建图导航[](#ros "Permalink to this heading")

## 新建一个雷达link[](#link "Permalink to this heading")

> 该link描述一个激光雷达

### 创建激光雷达URDF模型

在机器人的URDF文件中添加激光雷达link：

```xml
<!-- 激光雷达link定义 -->
<link name="laser_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>
  
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
  </collision>
  
  <inertial>
    <mass value="0.125"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- 将激光雷达连接到机器人底盘 -->
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser_link"/>
  <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
</joint>
```

### 加载Gazebo激光雷达插件

现在加载一个雷达插件让这个link被仿真为激光雷达：

```xml
<gazebo reference="laser_link">
  <sensor type="ray" name="laser_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="gazebo_ros_laser" filename="libgazebo_ros_laser.so">
      <topicName>/scan</topicName>
      <frameName>laser_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

**参数说明：**
- `samples`: 激光束数量，720表示0.5度一个采样点
- `min_angle/max_angle`: 扫描角度范围（弧度）
- `min/max range`: 最小/最大探测距离（米）
- `update_rate`: 扫描频率（Hz）
- `topicName`: 发布的ROS话题名称

### 启动和验证

启动launch文件，观察相应话题数据：

```bash
# 启动Gazebo仿真
roslaunch your_robot_description gazebo.launch

# 查看激光雷达话题
rostopic list | grep scan

# 查看激光雷达数据
rostopic echo /scan

# 使用rviz可视化
rosrun rviz rviz
```

在rviz中添加LaserScan显示，设置Fixed Frame为`laser_link`或`base_link`，Topic选择`/scan`。

![../../_images/4-1-4.png](http://dynamicx.top/_images/4-1-4.png)

## 过滤激光雷达数据[](#id1 "Permalink to this heading")

### LaserScan消息结构

在ROS中，激光雷达数据通过`sensor_msgs/LaserScan`消息类型发布：

```python
# LaserScan消息结构
Header header
float32 angle_min        # 扫描的起始角度（弧度）
float32 angle_max        # 扫描的结束角度（弧度）
float32 angle_increment  # 测量之间的角度增量（弧度）
float32 time_increment   # 测量之间的时间增量（秒）
float32 scan_time        # 扫描之间的时间（秒）
float32 range_min        # 最小测量距离（米）
float32 range_max        # 最大测量距离（米）
float32[] ranges         # 距离数据（米），inf表示无测量
float32[] intensities    # 强度数据
```

### 数据分区处理

现在通过代码获取雷达每隔一定距离的扫描最小值，将360度视野分为5个区域：

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserProcessor:
    def __init__(self):
        rospy.init_node('laser_processor')
        self.regions = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0,
        }
        self.sub = rospy.Subscriber('/scan', LaserScan, self.clbk_laser)
        
    def clbk_laser(self, msg):
        """
        将720个激光点分为5个区域
        每个区域144个点（720/5 = 144）
        """
        # 处理无效值（inf和nan）
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 10.0  # 将无穷大值设为10米
        ranges[np.isnan(ranges)] = 10.0  # 将nan值设为10米
        
        # 分区域计算最小值
        self.regions = {
            'right':  float(np.min(ranges[0:143])),      # 右侧 -180° 到 -108°
            'fright': float(np.min(ranges[144:287])),    # 右前 -108° 到 -36°
            'front':  float(np.min(ranges[288:431])),    # 正前 -36° 到 36°
            'fleft':  float(np.min(ranges[432:575])),    # 左前 36° 到 108°
            'left':   float(np.min(ranges[576:719])),    # 左侧 108° 到 180°
        }
        
        rospy.loginfo("Regions: right=%.2f, fright=%.2f, front=%.2f, fleft=%.2f, left=%.2f" % 
                     (self.regions['right'], self.regions['fright'], 
                      self.regions['front'], self.regions['fleft'], 
                      self.regions['left']))
        
    def get_regions(self):
        return self.regions

if __name__ == '__main__':
    try:
        processor = LaserProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

### 高级过滤技术

**中值滤波去噪：**

```python
from scipy.signal import medfilt

def filter_laser_data(ranges, kernel_size=5):
    """使用中值滤波器减少噪声"""
    filtered = medfilt(ranges, kernel_size=kernel_size)
    return filtered
```

**角度范围提取：**

```python
def get_range_at_angle(msg, target_angle):
    """
    获取指定角度的距离值
    target_angle: 目标角度（度）
    """
    angle_rad = np.deg2rad(target_angle)
    index = int((angle_rad - msg.angle_min) / msg.angle_increment)
    if 0 <= index < len(msg.ranges):
        return msg.ranges[index]
    return float('inf')
```

## 实现逻辑避障[](#id2 "Permalink to this heading")

将扫描获取的数据进行分析，得到要如何运动以规避障碍

![../../_images/4-1-5.png](http://dynamicx.top/_images/4-1-5.png)
### 避障决策逻辑

**决策树：**

```
如果前方有障碍物（距离 < 阈值）：
    如果左侧空旷 且 右侧有障碍：
        向左转
    如果右侧空旷 且 左侧有障碍：
        向右转
    如果左右都有障碍：
        后退并旋转
    如果左右都空旷：
        选择距离较远的一侧
否则：
    直行
```

### 完整避障实现

```python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        
        # 参数配置
        self.safe_distance = 0.8  # 安全距离（米）
        self.linear_speed = 0.3   # 前进速度
        self.angular_speed = 0.5  # 转向速度
        
        # ROS接口
        self.sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.regions = None
        self.rate = rospy.Rate(10)  # 10Hz控制频率
        
    def laser_callback(self, msg):
        """处理激光雷达数据"""
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 10.0
        ranges[np.isnan(ranges)] = 10.0
        
        # 分5个区域
        self.regions = {
            'right':  float(np.min(ranges[0:143])),
            'fright': float(np.min(ranges[144:287])),
            'front':  float(np.min(ranges[288:431])),
            'fleft':  float(np.min(ranges[432:575])),
            'left':   float(np.min(ranges[576:719])),
        }
        
    def decide_action(self):
        """基于激光雷达数据做出决策"""
        if self.regions is None:
            return None
            
        d = self.safe_distance
        regions = self.regions
        
        # 决策状态
        state_description = ''
        
        if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 1 - 前方通畅，直行'
            return 'forward'
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
            state_description = 'case 2 - 前方有障碍，左右都通畅'
            # 选择距离更远的一侧
            if regions['fleft'] > regions['fright']:
                return 'turn_left'
            else:
                return 'turn_right'
        elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 3 - 右前方有障碍，向左转'
            return 'turn_left'
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 4 - 左前方有障碍，向右转'
            return 'turn_right'
        elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
            state_description = 'case 5 - 前方和右前方有障碍，向左转'
            return 'turn_left'
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
            state_description = 'case 6 - 前方和左前方有障碍，向右转'
            return 'turn_right'
        elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 7 - 三面都有障碍，后退'
            return 'backward'
        elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
            state_description = 'case 8 - 左右都有障碍，直行'
            return 'forward'
        else:
            state_description = 'unknown case'
            rospy.logwarn(state_description)
            return 'stop'
            
    def execute_action(self, action):
        """执行运动指令"""
        msg = Twist()
        
        if action == 'forward':
            msg.linear.x = self.linear_speed
            msg.angular.z = 0
        elif action == 'turn_left':
            msg.linear.x = self.linear_speed * 0.5
            msg.angular.z = self.angular_speed
        elif action == 'turn_right':
            msg.linear.x = self.linear_speed * 0.5
            msg.angular.z = -self.angular_speed
        elif action == 'backward':
            msg.linear.x = -self.linear_speed * 0.3
            msg.angular.z = 0
        else:  # stop
            msg.linear.x = 0
            msg.angular.z = 0
            
        self.pub.publish(msg)
        
    def run(self):
        """主循环"""
        rospy.loginfo("Obstacle avoidance node started")
        
        while not rospy.is_shutdown():
            if self.regions is not None:
                action = self.decide_action()
                if action:
                    self.execute_action(action)
                    rospy.loginfo(f"Action: {action}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        avoider = ObstacleAvoidance()
        avoider.run()
    except rospy.ROSInterruptException:
        pass
```

### 使用方法

```bash
# 启动仿真环境
roslaunch your_robot gazebo.launch

# 启动避障节点
rosrun your_package obstacle_avoidance.py

# 可视化
rosrun rviz rviz
```
## 里程计和全局定位[](#id3 "Permalink to this heading")

> 在这部分中，我们将实现一个简单的导航算法，将机器人从任意点移动到所需的点。我们将使用状态机的概念来实现导航逻辑。在状态机中，有有限数量的状态表示系统的当前状况（或行为）

现在定义三个状态：

​ 表示机器人航向与所需航向相差超过阈值时的状态（由代码中的yaw\_precision\_表示）

​ 表示机器人具有正确的航向，但距离所需点的距离大于某个阈值时的状态（由代码中的dist\_precision\_表示）

​ 表示机器人方向正确并到达目的地时的状态。

![../../_images/4-1-6.png](http://dynamicx.top/_images/4-1-6.png)

### 发布里程计[](#id4 "Permalink to this heading")

在ROS中，发布里程计需要将当前机器人的位置信息发布到ROS系统中。可以通过以下步骤实现：

其中，robot\_x, robot\_y, robot\_yaw, linear\_velocity和angular\_velocity是机器人的位置和速度信息。

注意：里程计的发布频率较高，需要在一个while循环中不断地发布消息，同时需要保证循环周期的稳定性。

### 实现状态机[](#id5 "Permalink to this heading")

接收里程计数据并提取位置和偏航信息。里程数据以四元数编码方向信息。为了获得偏航，将四元数转换为欧拉角

此函数更改存储机器人状态信息的全局状态变量的值。

当机器人处于状态0（固定航向）时，执行此功能。首先，检查机器人的当前航向和所需航向。如果航向差大于阈值，机器人将被命令转向其位置。

当机器人处于状态1（直行）时，执行此功能。该状态发生在机器人修正偏航误差后。在此状态下，将机器人当前位置和期望位置之间的距离与阈值进行比较。如果机器人进一步远离所需位置，则命令其向前移动。如果当前位置更接近所需位置，则再次检查偏航是否存在错误，如果偏航与所需偏航值显著不同，则机器人进入状态0。

最终机器人实现了正确的航向和正确的位置。一旦处于这种状态，机器人就会停止。

## 沿墙行走[](#id6 "Permalink to this heading")

> 在这一部分中，我们将编写一个算法，让机器人沿着墙走。我们可以从上一部分继续，也可以从新项目开始。

## 实物雷达[](#id7 "Permalink to this heading")

## YDILAR[](#ydilar "Permalink to this heading")

## 安装Gmapping[](#gmapping "Permalink to this heading")

### Gmapping简介

Gmapping是一个基于粒子滤波的SLAM算法，使用激光雷达数据构建2D栅格地图。它的核心特点包括：

- **基于Rao-Blackwellized粒子滤波器（RBPF）**
- **使用激光雷达进行地图构建**
- **需要里程计或IMU提供位姿估计**
- **实时性好，适合小型室内环境**

#### 安装Gmapping包

```bash
# ROS Melodic/Noetic
sudo apt-get install ros-${ROS_DISTRO}-gmapping
sudo apt-get install ros-${ROS_DISTRO}-slam-gmapping

# 从源码编译
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/slam_gmapping.git
cd ~/catkin_ws
catkin_make
```

### 有里程计[](#id8 "Permalink to this heading")

#### 配置修改

在官方提供的sdk中为3600比默认值2048大．所以对应的到头文件中将值修改为：4096：

```bash
# 修改头文件
vim ~/catkin_rikirobot/src/slam_gmapping/openslam_gmapping/include/gmapping/scanmatcher/scanmatcher.h

# 找到以下行并修改
#define MAX_LASER_RANGE 4096  // 原来是2048
```

#### Gmapping核心参数配置

创建gmapping配置文件 `gmapping_params.yaml`：

```yaml
# Gmapping参数配置
gmapping:
  # 地图更新参数
  map_update_interval: 2.0          # 地图更新间隔(秒)
  maxUrange: 5.0                     # 激光雷达最大可用范围(米)
  maxRange: 10.0                     # 激光雷达最大测量范围(米)
  
  # 粒子滤波器参数
  particles: 30                      # 粒子数量(越多越精确但计算量大)
  
  # 扫描匹配参数
  linearUpdate: 0.5                  # 线性位移达到此值时更新(米)
  angularUpdate: 0.3                 # 角度变化达到此值时更新(弧度)
  temporalUpdate: -1.0               # 时间间隔更新，-1表示禁用
  
  # 似然场模型参数
  sigma: 0.05                        # 扫描匹配的高斯分布标准差
  kernelSize: 1                      # 搜索窗口大小
  lstep: 0.05                        # 线性搜索步长
  astep: 0.05                        # 角度搜索步长
  iterations: 5                      # 扫描匹配迭代次数
  
  # 重采样参数
  resampleThreshold: 0.5             # 粒子重采样阈值
  
  # 地图参数
  xmin: -10.0                        # 地图X最小值
  ymin: -10.0                        # 地图Y最小值
  xmax: 10.0                         # 地图X最大值
  ymax: 10.0                         # 地图Y最大值
  delta: 0.05                        # 地图分辨率(米/像素)
  
  # 里程计参数
  odom_frame: odom                   # 里程计坐标系
  base_frame: base_link              # 机器人基坐标系
  map_frame: map                     # 地图坐标系
```

#### `static_transform_publisher`[](#static-transform-publisher "Permalink to this heading")

> 静态发布器，`static_transform_publisher`主要用于发布静态的坐标系之间的变换，而`robot_state_publisher`则用于发布机器人的动态运动信息

#### TF坐标变换概念

假设我们有一个机器人系统，包括机器人底座、激光雷达和相机，它们的坐标系关系如下：

```
map -> odom -> base_link -> laser_link
                          -> camera_link
```

**坐标系说明：**
- `map`: 全局地图坐标系（固定）
- `odom`: 里程计坐标系（相对于起点）
- `base_link`: 机器人底盘中心坐标系
- `laser_link`: 激光雷达坐标系
- `camera_link`: 相机坐标系

#### 发布静态TF变换示例

假设我们需要将激光雷达坐标系与机器人底座坐标系之间的静态变换发布出去，可以使用`static_transform_publisher`来实现。假设激光雷达在机器人底座的前方0.5米处，离机器人底座中心轴线偏离20度，可以使用如下命令发布变换：

```bash
# 方法1: 使用命令行
rosrun tf static_transform_publisher 0.5 0 0.1 0 0 0.349 base_link laser_link 100

# 方法2: 在launch文件中
```

```xml
<launch>
  <!-- 发布激光雷达到base_link的静态变换 -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_laser"
        args="0.5 0 0.1 0 0 0.349 base_link laser_link 100" />
        
  <!-- 或者使用static_transform_publisher节点 (ROS Kinetic+) -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_laser_tf2"
        args="0.5 0 0.1 0 0 0.349 base_link laser_link" />
</launch>
```

**参数说明：**
- `x y z`: 平移变换 (米)，这里是 (0.5, 0, 0.1)
  - x=0.5: 激光雷达在底盘前方0.5米
  - y=0: 不偏左右
  - z=0.1: 高出底盘中心0.1米
- `roll pitch yaw`: 旋转变换 (弧度)，这里是 (0, 0, 0.349)
  - roll=0: 不绕X轴旋转
  - pitch=0: 不绕Y轴旋转
  - yaw=0.349: 绕Z轴旋转20度 (20° = 0.349弧度)
- `frame_id`: 父坐标系 (`base_link`)
- `child_frame_id`: 子坐标系 (`laser_link`)
- `period_in_ms`: 发布周期 (毫秒)，这里是100ms

#### 完整的Gmapping Launch文件

```xml
<launch>
  <!-- 激光雷达驱动 -->
  <node name="laser_driver" pkg="ydlidar_ros" type="ydlidar_node" output="screen">
    <param name="port" value="/dev/ttyUSB0"/>
    <param name="baudrate" value="128000"/>
    <param name="frame_id" value="laser_link"/>
    <param name="angle_min" value="-180"/>
    <param name="angle_max" value="180"/>
    <param name="range_min" value="0.1"/>
    <param name="range_max" value="12.0"/>
  </node>
  
  <!-- 发布TF变换 -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_laser"
        args="0.15 0 0.1 0 0 0 base_link laser_link 50"/>
  
  <!-- Gmapping SLAM -->
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
    <remap from="scan" to="/scan"/>
    <rosparam file="$(find your_package)/config/gmapping_params.yaml" command="load"/>
  </node>
  
  <!-- Rviz可视化 -->
  <node pkg="rviz" type="rviz" name="rviz" 
        args="-d $(find your_package)/rviz/gmapping.rviz"/>
</launch>
```

这样就可以在ROS系统中发布一个名为`/base_link`到`/laser_link`的坐标系变换了，其他节点可以通过tf库订阅该变换，并在不同坐标系下进行数据转换和计算。

#### 运行Gmapping建图

```bash
# 启动机器人和Gmapping
roslaunch your_robot gmapping.launch

# 启动遥控节点控制机器人运动
roslaunch your_robot teleop_keyboard.launch

# 保存地图
rosrun map_server map_saver -f ~/maps/my_map
```

建图技巧：
1. **缓慢移动**：避免快速转向导致里程计误差累积
2. **多次覆盖**：重复扫描同一区域提高地图精度
3. **闭环**：尽量走回起点形成闭环，帮助修正误差
4. **丰富特征**：在特征丰富的环境中建图效果更好

### 无里程计[](#id9 "Permalink to this heading")

无里程计要使用gmapping和激光雷达进行建图，可以按照以下步骤进行：

1.  安装ROS和gmapping包：首先需要安装ROS和gmapping包，可以使用命令
    
    `sudo apt-get install ros-<distro>-gmapping`
    
2.  下载无里程计工具包
    

​ [https://github.com/ccny-ros-pkg/scan\_tools.git](https://github.com/ccny-ros-pkg/scan_tools.git)

3.  修改demo\_gmapping.launch
    

4.  现在可以运行launch了
    
5.  保存地图：当地图建立完成后，可以使用`rosrun map_server map_saver -f <mapname>`来保存地图。其中`<mapname>`是地图的文件名，例如`map`。
    

#### `laser_scan_matcher`[](#laser-scan-matcher "Permalink to this heading")

对没有里程计的设备使用，生成仿真里程计。包中提供了一个节点，它可以接收激光雷达数据并使用扫描匹配算法将机器人在地图中的位置进行估计。该软件包还提供了一些参数配置选项，可以用于调整扫描匹配算法的性能和精度

> 这个包发布里程计，也就是底盘base\_link到odom的位置姿态，同时静态发布器static\_transform\_publisher发布雷达坐标系到base\_link的变换。
> 
> gmapping通过参数：
> 
> 在/tf上获取机器人底盘坐标系(默认base\_link)和世界坐标系(默认odom)关系，并利用雷达话题scan\_topic获取点云，在代码中会获取坐标系转换：
> 
> ```
> //在laserCallback()函数
>  tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
>  tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));
> ```

## SLAM[](#slam "Permalink to this heading")

### SLAM基础概念

**SLAM (Simultaneous Localization and Mapping)** 即同时定位与地图构建，是移动机器人在未知环境中导航的关键技术。

#### SLAM核心问题

1. **定位（Localization）**：机器人在地图中确定自身位置
2. **建图（Mapping）**：根据传感器数据构建环境地图  
3. **闭环检测（Loop Closure）**：识别已访问位置，消除累积误差

#### 主流激光SLAM算法对比

| 特性 | Gmapping | Hector SLAM | Cartographer | Karto SLAM |
|------|----------|-------------|--------------|------------|
| **算法基础** | Rao-Blackwellized粒子滤波 | 高频扫描匹配 | 图优化 | 图优化 |
| **是否需要里程计** | 需要 | 不需要 | 可选 | 需要 |
| **实时性** | 好 | 很好 | 好 | 中等 |
| **地图精度** | 中等 | 中等 | 高 | 高 |
| **计算资源** | 中等 | 低 | 高 | 中高 |
| **闭环检测** | 无 | 无 | 有 | 有 |
| **适用场景** | 小型室内 | 救援/快速建图 | 大型复杂环境 | 中型室内 |
| **地图类型** | 2D栅格 | 2D栅格 | 2D/3D | 2D栅格 |

### 算法详细对比

#### Gmapping
**优点：**
- 实时性能好，CPU占用低
- 对里程计要求不高，有一定容错性
- 小型室内环境建图效果好
- 配置简单，容易上手

**缺点：**
- 无闭环检测，长时间运行误差累积
- 粒子数量影响精度和性能
- 大场景下效果一般

**适用场景：** 家庭、办公室等小型室内环境

#### Cartographer
**优点：**
- 强大的闭环检测和位姿图优化
- 支持2D和3D SLAM
- 可处理大规模环境
- 地图精度高

**缺点：**
- 计算资源需求高
- 配置复杂，参数调优难度大
- 实时性略低于Gmapping

**适用场景：** 仓库、商场等大型复杂环境

对比资料：[SLAM算法详细对比](https://blog.csdn.net/qq_40695642/article/details/128472360)

## rtabmap\_ros[](#rtabmap-ros "Permalink to this heading")
### RTAB-Map简介

RTAB-Map（Real-Time Appearance-Based Mapping）是一个RGB-D、立体视觉和激光雷达SLAM方法，用于大规模、长期的在线操作。它集成了外观基础的闭环检测和图优化。

#### 核心特性

- **多传感器支持**：RGB-D相机（Kinect、RealSense）、双目相机、激光雷达、IMU
- **内存管理**：通过工作内存（WM）和长期内存（STM/LTM）机制，适应有限计算资源
- **闭环检测**：基于视觉词袋（Bag-of-Words）的外观识别
- **3D地图**：生成点云地图和2D占用栅格地图
- **实时性能**：适合在线SLAM应用

### 安装RTAB-Map

```bash
# 安装二进制包（推荐）
sudo apt-get install ros-${ROS_DISTRO}-rtabmap-ros

# 验证安装
rospack find rtabmap_ros

# 从源码编译（可选）
cd ~/catkin_ws/src
git clone https://github.com/introlab/rtabmap.git
git clone https://github.com/introlab/rtabmap_ros.git
cd ~/catkin_ws
catkin_make
```

### 2D激光雷达配置

使用激光雷达进行2D SLAM的launch文件：

```xml
<launch>
  <!-- 激光雷达 -->
  <node pkg="ydlidar_ros" type="ydlidar_node" name="ydlidar_node" output="screen"/>

  <!-- 里程计（从轮式编码器或视觉里程计） -->
  <node pkg="your_robot" type="odom_publisher" name="odom_publisher"/>

  <!-- RTAB-Map -->
  <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen">
    <param name="frame_id" type="string" value="base_link"/>
    <param name="subscribe_scan" type="bool" value="true"/>
    <param name="subscribe_depth" type="bool" value="false"/>
    
    <!-- RTAB-Map输入 -->
    <remap from="scan" to="/scan"/>
    <remap from="odom" to="/odom"/>
    
    <!-- RTAB-Map核心参数 -->
    <param name="Rtabmap/DetectionRate" type="string" value="1"/>
    <param name="RGBD/Enabled" type="string" value="false"/>
    <param name="Mem/IncrementalMemory" type="string" value="true"/>
    <param name="Mem/InitWMWithAllNodes" type="string" value="false"/>
    
    <!-- 优化参数 -->
    <param name="Optimizer/Strategy" type="string" value="1"/> <!-- g2o -->
    <param name="Optimizer/Iterations" type="string" value="20"/>
    
    <!-- 2D设置 -->
    <param name="Reg/Force3DoF" type="string" value="true"/>
    <param name="Grid/FromDepth" type="string" value="false"/>
  </node>
  
  <!-- 地图可视化 -->
  <node pkg="rtabmap_ros" type="rtabmapviz" name="rtabmapviz" output="screen">
    <param name="frame_id" type="string" value="base_link"/>
    <param name="subscribe_scan" type="bool" value="true"/>
    <remap from="scan" to="/scan"/>
    <remap from="odom" to="/odom"/>
  </node>
  
  <!-- Rviz -->
  <node type="rviz" pkg="rviz" name="rviz" 
        args="-d $(find rtabmap_ros)/launch/config/demo_robot_mapping.rviz"/>
</launch>
```

### RGB-D相机配置

使用深度相机进行视觉SLAM：

```xml
<launch>
  <!-- RealSense D435i -->
  <include file="$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="align_depth" value="true"/>
    <arg name="enable_gyro" value="true"/>
    <arg name="enable_accel" value="true"/>
  </include>

  <!-- RTAB-Map -->
  <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="--delete_db_on_start">
    <param name="frame_id" type="string" value="base_link"/>
    
    <param name="subscribe_depth" type="bool" value="true"/>
    <param name="subscribe_rgb" type="bool" value="true"/>
    <param name="subscribe_scan" type="bool" value="false"/>
    
    <remap from="rgb/image" to="/camera/color/image_raw"/>
    <remap from="depth/image" to="/camera/aligned_depth_to_color/image_raw"/>
    <remap from="rgb/camera_info" to="/camera/color/camera_info"/>
    
    <!-- 视觉里程计参数 -->
    <param name="Odom/Strategy" type="string" value="0"/> <!-- Frame-to-Map -->
    <param name="Odom/ResetCountdown" type="string" value="1"/>
    <param name="OdomF2M/MaxSize" type="string" value="1000"/>
    
    <!-- 闭环检测 -->
    <param name="Kp/DetectorStrategy" type="string" value="6"/> <!-- GFTT/BRIEF -->
    <param name="Vis/MinInliers" type="string" value="12"/>
    <param name="RGBD/OptimizeFromGraphEnd" type="string" value="false"/>
  </node>
  
  <!-- 可视化 -->
  <node pkg="rtabmap_ros" type="rtabmapviz" name="rtabmapviz"/>
</launch>
```

### 关键参数说明

#### 内存管理参数

```xml
<!-- 增量式建图（在线SLAM） -->
<param name="Mem/IncrementalMemory" type="string" value="true"/>

<!-- 工作内存大小限制（节点数量） -->
<param name="Rtabmap/TimeThr" type="string" value="0"/> <!-- 0=无限制 -->
<param name="Rtabmap/MemoryThr" type="string" value="0"/> <!-- 0=无限制 -->

<!-- STM大小（短期记忆） -->
<param name="Mem/STMSize" type="string" value="30"/>
```

#### 闭环检测参数

```xml
<!-- 检测率（Hz） -->
<param name="Rtabmap/DetectionRate" type="string" value="1"/>

<!-- 闭环检测阈值 -->
<param name="Mem/RehearsalSimilarity" type="string" value="0.30"/>
<param name="Kp/MaxFeatures" type="string" value="400"/>

<!-- 特征提取器：0=SURF, 6=GFTT, 8=FAST -->
<param name="Kp/DetectorStrategy" type="string" value="6"/>
```

#### 优化参数

```xml
<!-- 优化器类型：0=TORO, 1=g2o, 2=GTSAM -->
<param name="Optimizer/Strategy" type="string" value="1"/>

<!-- 优化迭代次数 -->
<param name="Optimizer/Iterations" type="string" value="20"/>

<!-- Robust核函数 -->
<param name="Optimizer/Robust" type="string" value="true"/>
```

### 使用RTAB-Map建图

```bash
# 1. 删除之前的数据库（新建图）
roslaunch your_robot rtabmap_mapping.launch

# 2. 控制机器人探索环境
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

# 3. 暂停建图（可选）
rosservice call /rtabmap/pause

# 4. 恢复建图
rosservice call /rtabmap/resume

# 5. 触发全局优化
rosservice call /rtabmap/trigger_new_map

# 6. 保存地图
rosservice call /rtabmap/save_map "filename: '~/maps/my_map.db'"
```

### 数据库管理

```bash
# 查看数据库信息
rtabmap-databaseViewer ~/.ros/rtabmap.db

# 导出点云
rtabmap-export --cloud output.ply input.db

# 导出2D栅格地图
rtabmap-export --grid_map output.pgm input.db

# 合并多个数据库
rtabmap-merge output.db input1.db input2.db
```

### 定位模式（Localization Mode）

使用已有地图进行定位：

```xml
<launch>
  <!-- 加载已有地图 -->
  <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen">
    <param name="Mem/IncrementalMemory" type="string" value="false"/>
    <param name="Mem/InitWMWithAllNodes" type="string" value="true"/>
    <param name="database_path" type="string" value="$(find your_pkg)/maps/my_map.db"/>
    
    <!-- 其他参数与建图相同 -->
    <remap from="scan" to="/scan"/>
    <remap from="odom" to="/odom"/>
  </node>
</launch>
```

### RTAB-Map与其他SLAM对比

| 特性 | RTAB-Map | Gmapping | Cartographer |
|------|----------|----------|--------------|
| 传感器类型 | RGB-D/双目/激光 | 激光雷达 | 激光雷达/IMU |
| 3D建图 | ✅ 支持 | ❌ 不支持 | ⚠️ 有限支持 |
| 闭环检测 | ✅ 外观+几何 | ❌ 无 | ✅ 几何 |
| 内存管理 | ✅ 智能管理 | ⚠️ 全存储 | ⚠️ 全存储 |
| 实时性 | ⚠️ 中等 | ✅ 优秀 | ✅ 优秀 |
| 大场景 | ✅ 适合 | ❌ 受限 | ✅ 适合 |

### 常见问题与解决

#### 1. 闭环检测不准确

```xml
<!-- 增加特征点数量 -->
<param name="Kp/MaxFeatures" type="string" value="600"/>
<!-- 提高内联点要求 -->
<param name="Vis/MinInliers" type="string" value="20"/>
```

#### 2. 内存占用过高

```xml
<!-- 限制工作内存 -->
<param name="Rtabmap/TimeThr" type="string" value="700"/> <!-- ms -->
<param name="Rtabmap/MemoryThr" type="string" value="300"/> <!-- MB -->
```

#### 3. 地图漂移

```xml
<!-- 强制3自由度约束 -->
<param name="Reg/Force3DoF" type="string" value="true"/>
<!-- 增加ICP迭代 -->
<param name="Icp/Iterations" type="string" value="30"/>
```
## cartographer[](#cartographer "Permalink to this heading")

### Cartographer简介

Google Cartographer 是一个跨平台、实时的 2D 和 3D SLAM 库，提供了ROS集成支持。

#### 核心特点

- **实时闭环检测**：通过分支定界扫描匹配实现闭环
- **位姿图优化**：使用Sparse Pose Adjustment进行全局优化
- **多传感器融合**：支持激光雷达、IMU、里程计融合
- **亚地图概念**：将大地图分解为多个小的亚地图管理

### 安装Cartographer

```bash
# 安装依赖
sudo apt-get install -y \
    google-mock \
    libboost-all-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.2-dev \
    libprotobuf-dev \
    libsuitesparse-dev \
    libwebp-dev \
    ninja-build \
    protobuf-compiler \
    python-sphinx

# 安装Cartographer
sudo apt-get install ros-${ROS_DISTRO}-cartographer
sudo apt-get install ros-${ROS_DISTRO}-cartographer-ros

# 或从源码编译
cd ~/catkin_ws/src
git clone https://github.com/cartographer-project/cartographer.git
git clone https://github.com/cartographer-project/cartographer_ros.git
cd ~/catkin_ws
catkin_make_isolated --install --use-ninja
```

### Cartographer配置文件

创建 `cartographer_2d.lua` 配置文件：

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimization_problem.huber_scale = 1e1

return options
```

### Cartographer Launch文件

```xml
<launch>
  <!-- 激光雷达 -->
  <node name="laser_driver" pkg="ydlidar_ros" type="ydlidar_node"/>
  
  <!-- Cartographer -->
  <node name="cartographer_node" pkg="cartographer_ros"
        type="cartographer_node" args="
            -configuration_directory $(find your_package)/config
            -configuration_basename cartographer_2d.lua"
        output="screen">
    <remap from="scan" to="/scan"/>
    <remap from="odom" to="/odom"/>
  </node>
  
  <!-- Cartographer占用栅格地图 -->
  <node name="cartographer_occupancy_grid_node" 
        pkg="cartographer_ros"
        type="cartographer_occupancy_grid_node"
        args="-resolution 0.05"/>
  
  <!-- Rviz -->
  <node name="rviz" pkg="rviz" type="rviz"
        args="-d $(find your_package)/rviz/cartographer.rviz"/>
</launch>
```

### 使用Cartographer建图

```bash
# 启动Cartographer
roslaunch your_robot cartographer.launch

# 控制机器人移动建图
roslaunch your_robot teleop.launch

# 结束建图并保存
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${HOME}/maps/cartographer_map.pbstream'}"

# 将pbstream转换为pgm/yaml格式
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename=${HOME}/maps/cartographer_map.pbstream \
  -map_filestem=${HOME}/maps/my_map
```

### Cartographer与Gmapping对比

**Cartographer优势：**
1. **闭环检测**：自动识别回到之前位置，消除累积误差
2. **全局优化**：持续优化整个轨迹和地图
3. **更大场景**：适合大型环境建图
4. **更高精度**：地图质量更好

**Gmapping优势：**
1. **更低延迟**：实时响应更快
2. **资源占用少**：CPU和内存需求低
3. **配置简单**：参数少，易于调试
4. **小场景高效**：在小环境中性能出色

**选择建议：**
- 小型室内（<200㎡）：Gmapping
- 大型环境（>200㎡）：Cartographer  
- 需要高精度地图：Cartographer
- 计算资源受限：Gmapping

### 常见问题与解决

#### 1. Cartographer地图漂移

**原因：** 里程计误差大或激光数据质量差

**解决：**
```lua
-- 增加扫描匹配权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.
```

#### 2. 闭环检测失败

**解决：**
```lua
-- 降低闭环检测阈值
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
```

#### 3. CPU占用过高

**解决：**
```lua
-- 减少亚地图数量和分辨率
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1
```

cartographer和gmapping都是SLAM算法的实现，用于构建地图和定位机器人。它们的主要不同在于：

- **算法原理**：gmapping使用粒子滤波，cartographer使用图优化
- **闭环检测**：cartographer有强大的闭环检测，gmapping没有
- **适用规模**：gmapping适合小场景，cartographer适合大场景
- **计算资源**：gmapping资源占用少，cartographer需要更多计算资源

另外，还有一些其他的SLAM算法，如Hector和Karto，它们也有各自的优缺点和适用场景。你可以根据你的需求和条件选择合适的算法。