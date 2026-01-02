# 无人机 VIO + 光流定位详解（原理、工程接入与调参）

> 目标：把“无人机在没有/弱 GPS 环境下如何定位”这件事讲清楚，并给出在 **ROS + MAVROS + PX4** 场景里常见的接入方法、关键参数和排障清单。

---

## 0. 先建立整体认识：你到底在做哪一种“定位”？

无人机的“定位/导航”通常分成三个层次：

1. **状态估计（State Estimation）**：估计位置/速度/姿态（EKF/UKF/优化）。PX4里通常是 EKF2。
2. **定位源（Localization Source）**：提供“位姿/速度”观测，例如 GPS、VIO、光流、UWB、激光里程计。
3. **控制闭环（Control）**：位置/速度控制器（OFFBOARD、POSCTL 等），对估计出的状态闭环。

你学习的 **VIO + 光流定位**，本质是给飞控的 EKF 提供观测（Vision / OpticalFlow / Range 等），从而在室内或 GPS 差的地方“稳定定点/导航”。

---

## 1. VIO（Visual-Inertial Odometry）是什么？

### 1.1 定义

**VIO** = 用相机（视觉）+ IMU（惯性）融合，输出高频姿态与中高频位置/速度（通常 50~200Hz）。

它和“纯视觉 VO”最大的区别：
- IMU提供短时高频、可观测重力方向与角速度，能显著增强高速运动、弱纹理、动态模糊场景的鲁棒性。

### 1.2 VIO的两种主流框架

1) **滤波式（Filter-based）**：典型是 MSCKF 系列。
- 优点：实时性强、计算量较稳定。
- 缺点：精度和一致性受线性化影响较大。

2) **优化式（Optimization / Sliding Window）**：典型是 OKVIS、VINS(-Fusion)、ORB-SLAM3（带IMU）。
- 优点：精度高、可加入回环/地图。
- 缺点：计算量更大，参数/同步/标定更敏感。

### 1.3 VIO能“绝对定位”吗？

通常 **VIO输出的是相对里程计（odometry）**：
- 原点取决于启动时刻（或地图锚点）
- 长时间会有漂移（drift）

如果加入 **回环（Loop Closure）/地图重定位**，可以抑制漂移，实现“长期稳定”。但对于很多无人机 OFFBOARD / 悬停任务，短中时间尺度 VIO 里程计已经够用。

---

## 2. 光流定位（Optical Flow）是什么？

### 2.1 定义

**光流**是在图像平面上估计像素运动速度（或位移），在已知/估计高度 $h$ 的情况下，可换算为地面平面的速度（或位移）。

在无人机上常见用途：
- **室内/低空悬停**：替代/补充 GPS 水平速度观测
- **近地速度估计**：在纹理足够时非常稳定

### 2.2 光流=速度，不是位置

很多人误解“光流定位=位置”。更准确是：
- 光流主要给出 **水平速度观测 $v_x, v_y$**（对地）
- 要得到位置，需要再积分 + 融合其它信息（IMU/高度计/视觉定位）

### 2.3 高度的重要性

光流从像素速度换算到线速度，强依赖高度：

$$
v \approx \frac{h}{f} \cdot \dot{u}
$$

其中 $f$ 是等效焦距，$\dot{u}$ 是像素运动速度。高度 $h$ 的误差会直接变成速度尺度误差。

因此工程上光流几乎总是配合：
- **测距（Rangefinder/Lidar/超声）** 或
- **可靠的高度估计（baro + IMU + 地面效应修正）**

---

## 3. VIO vs 光流：什么时候用哪个？能否叠加？

### 3.1 对比

| 维度 | VIO | 光流 |
|---|---|---|
| 输出 | 位姿/速度/偏置（取决于算法） | 主要是水平速度（有时含质量指标） |
| 纹理依赖 | 中等（特征/直接法） | 强（地面纹理） |
| 高度依赖 | 不一定（有尺度观测来自IMU） | 强依赖高度 |
| 适用高度 | 更宽（取决于相机视野/算法） | 低空更好（1~10m典型） |
| 漂移 | 有（可回环抑制） | 积分会漂移；通常只做速度观测 |
| 典型用途 | 室内导航、全向飞行、SLAM | 悬停、定点、低空速度估计 |

### 3.2 能否叠加？

可以，但要注意“重复观测”和“权重冲突”。常见组合：
- **VIO做主定位（位置/姿态），光流作为低空速度增强**
- 或者仅使用 VIO，不用光流（VIO质量足够时）

在 PX4 EKF2 中，本质是给 EKF 提供多个外部观测源：Vision（外部里程计）+ OpticalFlow（光流）+ Range（高度），由 EKF 权衡融合。

---

## 4. 坐标系与时间同步：90%问题的根源

### 4.1 常见坐标系

在 ROS / MAVROS / PX4 里你会遇到：
- ROS常用：`map`, `odom`, `base_link`（FLU：前左上）
- 航空常用：NED（北东地）或 FRD（前右下）

关键点：**PX4内部使用 NED/FRD 体系**，而 ROS 默认是 ENU/FLU 体系（很多包如此）。

因此接入时最要命的是：
- 你发给飞控的姿态/速度/位置是哪个系？
- MAVROS插件是否帮你转换了？（很多 topic 已经在 ROS系表达，但要确认）

建议：在系统设计里明确写一张“坐标系合同”：
- VIO输出在 `odom`（ENU）
- 发送给 `/mavros/vision_pose/pose`（MAVROS会做 ENU->NED 或按配置处理）
- EKF融合使用 NED 内部状态

### 4.2 时间同步

VIO/光流是强时间敏感系统。常见坑：
- 相机与IMU硬件不同步（或驱动时间戳不准）
- ROS里 `header.stamp` 用了 `Time::now()` 而不是传感器时间
- 网络转发延迟大

经验法则：
- **所有测量必须使用传感器采样时刻的时间戳**
- IMU与相机最好是硬同步或同一设备输出（如 Realsense、ZED、某些VIO套件）
- 在 PX4 中通常也会做 timesync（MAVROS timesync）

---

## 5. VIO典型数据链路（ROS为例）

以常见 VIO（如 VINS-Fusion/ORB-SLAM3/OKVIS 等）为例，典型输出：
- `nav_msgs/Odometry`（位姿 + 速度）
- 或 `geometry_msgs/PoseStamped`（位姿）
- TF：`map -> odom -> base_link`

工程要点：
1. **相机-IMU外参（$T_{cam}^{imu}$）**：必须准确，否则姿态与速度会发散或漂移很快。
2. **IMU噪声参数**：与真实IMU一致（随机游走、角速度/加速度噪声）。
3. **特征质量**：曝光、快门时间、运动模糊、镜头畸变校正。

---

## 6. 光流典型数据链路

光流来源通常有两类：

1) **飞控/光流模块内部计算**
- 例如 PX4 + PMW3901 等模块
- 优点：链路短、延迟小
- 缺点：算法/参数受硬件限制

2) **机载计算机（ROS）计算光流并通过 MAVROS 输入飞控**
- 优点：可以用更复杂算法，易调试
- 缺点：延迟/同步更难

无论哪类，通常还需要 **高度输入**（rangefinder）才能变成对地速度观测。

---

## 7. 接入 PX4 + MAVROS：常用话题（实用版）

### 7.1 VIO / 视觉里程计输入

常用话题（根据 MAVROS 插件）：
- `/mavros/vision_pose/pose`（`geometry_msgs/PoseStamped`）
- `/mavros/odometry/in`（`nav_msgs/Odometry`）
- `/mavros/vision_speed/speed_vector`（`geometry_msgs/TwistStamped`，视版本/配置）

实践建议：
- 如果你有速度估计，优先使用 odometry（包含速度与协方差），EKF更好融合。
- 一定要提供合理的协方差（covariance），否则 EKF 可能“过度相信/完全不信”。

### 7.2 光流输入

常见：
- `/mavros/px4flow/raw/optical_flow_rad`（如果使用 px4flow 插件/设备）
- 或通过对应插件把光流量/质量输入给 PX4

光流通常还要配合：
- 距离：`/mavros/distance_sensor/*` 或 rangefinder 数据

### 7.3 最小可用组合（室内悬停）

如果你的目标是“室内稳定定点/慢速移动”：
- 方案A（更通用）：**VIO (pose+vel) + IMU + 气压计**（可选加 range）
- 方案B（更轻量）：**光流（水平速度） + rangefinder（高度） + IMU**

---

## 8. OFFBOARD / 定点控制为什么经常失败？

常见原因并非控制器写错，而是估计/输入链路不满足前提：

1. **OFFBOARD模式需要持续 setpoint（≥2Hz，通常 20Hz）**
2. **EKF没有“有效的水平位置/速度”来源**，导致位置控制不可用
3. **视觉输入坐标系错（ENU/NED、FRD/FLU）**，表现为“往反方向飞/发散”
4. **时间戳乱/延迟大**，EKF观测被判为过期
5. **协方差不合理**：太小=过度相信（抖动/发散），太大=几乎不用（漂移大）

---

## 9. 调参思路（不依赖具体某个VIO算法）

### 9.1 VIO侧

优先级从高到低：
1. **相机标定**：内参/畸变；滚动快门会影响高速运动
2. **相机-IMU外参**：误差会导致融合发散
3. **IMU噪声**：用 datasheet + 实测 Allan variance 更稳
4. **特征跟踪参数**：最少特征数、金字塔层级、RANSAC阈值
5. **关键帧/窗口大小**：影响延迟与鲁棒性

判断VIO是否健康的指标：
- 运动时轨迹是否平滑
- 静止时速度是否接近 0
- 估计的IMU bias 是否缓慢变化（而不是乱跳）

### 9.2 光流侧

核心变量：
- **高度测量质量**（range噪声、地面反射、倾角补偿）
- **地面纹理/光照**（纯白地板、反光地面会直接失效）
- **相机朝下安装角度**（需标定/补偿）
- **质量指标（quality）阈值**：低质量时不要硬融

经验：
- 光流在 0.5~3m 高度常见最稳；高度太高像素运动太小、噪声主导。

---

## 10. 诊断与排障清单（强烈建议按顺序）

### 10.1 先看链路是否通

```bash
rostopic echo /mavros/state
rostopic hz /mavros/vision_pose/pose
rostopic hz /mavros/odometry/in
```

要点：
- `/mavros/state.connected` 必须为 true
- VIO输入频率应稳定（例如 30Hz/50Hz）

### 10.2 检查坐标系方向

做一个简单测试：
- 机体向前移动 1m
- 观察 VIO 的 x/y 是否按预期变化
- 观察发给飞控的 vision_pose 是否同向

如果出现“向前走，估计往后”：几乎必然是坐标系/外参问题。

### 10.3 检查时间戳

```bash
rostopic echo -n 1 /mavros/odometry/in/header
```

看 `stamp` 是否合理（不应跳变、不应比当前时间落后太久）。

### 10.4 检查高度与光流匹配

- `range` 是否随高度变化且稳定
- 光流质量是否高（地面纹理足够、无过曝）

---

## 11. 一个“最小可用”的ROS发布模板（VIO -> MAVROS）

如果你的VIO输出是 `nav_msgs/Odometry`（推荐），可直接转发到 `/mavros/odometry/in`，核心注意点：
- 使用原始时间戳
- 填协方差
- frame_id / child_frame_id 一致

示例（伪代码，展示要点）：

```python
import rospy
from nav_msgs.msg import Odometry

pub = rospy.Publisher('/mavros/odometry/in', Odometry, queue_size=10)

def vio_cb(vio_odom: Odometry):
	out = Odometry()
	out.header = vio_odom.header          # 关键：保留时间戳
	out.child_frame_id = vio_odom.child_frame_id
	out.pose = vio_odom.pose
	out.twist = vio_odom.twist

	# 关键：协方差要合理（不要全0）
	# out.pose.covariance = ...
	# out.twist.covariance = ...

	pub.publish(out)

rospy.init_node('vio_to_mavros')
rospy.Subscriber('/vio/odometry', Odometry, vio_cb, queue_size=50)
rospy.spin()
```

---

## 12. 学习路线建议（按性价比排序）

1) 先把坐标系、时间戳、协方差概念搞清楚（这是接入的地基）
2) 选择一套成熟 VIO（VINS-Fusion/ORB-SLAM3/OKVIS等）跑通并输出稳定 odometry
3) 用 MAVROS 把 odometry 输入 PX4，先实现室内悬停（OFFBOARD 或 POSCTL）
4) 再加入光流（如果你确实需要低空速度增强/冗余）
5) 最后再考虑回环、地图、多传感器（GPS/UWB/LiDAR）

---

## 13. 常见问答

### Q1：为什么VIO看起来“轨迹对”，但飞控就是不稳？
通常是**时间戳延迟/坐标系错/协方差不合理**。VIO轨迹“视觉上对”不代表满足 EKF 的融合前提。

### Q2：光流能不能单独做到室内位置控制？
光流本质更适合作为**速度观测**。单靠光流积分会漂移；要想做“位置控制”，通常需要额外约束（VIO/UWB/回环/地图）。

### Q3：室内推荐 VIO 还是 光流？
想要“能到处飞、能走廊穿行、能绕障”：优先 **VIO**。
只需要“低空悬停/缓慢移动”，且有良好地面纹理和高度：光流方案很省算力。

---

**最后更新**：2026年1月2日

