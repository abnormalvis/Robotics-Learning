# 麦轮底盘控制器测试指南

## 📦 编译状态
✅ **编译成功！** (2025-12-25)

## 🚀 快速测试

### 方法1：使用测试脚本（推荐）
```bash
cd /home/idris/As-my-see/robotics-learning/src/mecanum_ppo_avoidance
./test_controller.sh
```

### 方法2：手动分步测试

#### 步骤1：Source环境
```bash
source /home/idris/As-my-see/robotics-learning/devel/setup.bash
```

#### 步骤2：启动Gazebo和RViz显示
```bash
roslaunch mecanum_ppo_avoidance display_with_tf.launch
```
等待Gazebo完全启动后，再打开新终端执行下一步。

#### 步骤3：加载控制器（新终端）
```bash
source /home/idris/As-my-see/robotics-learning/devel/setup.bash
roslaunch mecanum_ppo_avoidance load_controller.launch
```

## 🎮 测试控制器

### 1. 发送速度命令
```bash
# 前进
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

# 向左平移（麦轮特性）
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.5
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10

# 旋转
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5" -r 10
```

### 2. 使用键盘遥控
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 3. 查看话题和状态
```bash
# 查看所有话题
rostopic list

# 查看里程计
rostopic echo /odom

# 查看关节状态
rostopic echo /joint_states

# 查看控制器状态
rostopic echo /controller/hero_chassis_controller/state
```

## 📊 检查项目

### ✅ 编译检查
- [x] CMakeLists.txt 配置正确
- [x] package.xml 依赖完整
- [x] 插件描述文件存在
- [x] 源代码编译成功

### 🔍 运行时检查
执行以下命令验证控制器是否正常工作：

```bash
# 1. 检查控制器是否加载
rosservice call /controller_manager/list_controllers

# 2. 检查TF树
rosrun tf view_frames

# 3. 检查节点
rosnode list

# 4. 检查话题
rostopic hz /odom
rostopic hz /joint_states
```

## 🐛 故障排查

### 问题1：控制器未加载
**症状**: `rosservice call /controller_manager/list_controllers` 没有输出
**解决**: 
1. 检查 Gazebo 插件是否加载
2. 查看 params.yaml 配置
3. 检查日志：`tail -f logs/load_controller.log`

### 问题2：机器人不动
**症状**: 发送 /cmd_vel 但机器人无反应
**解决**:
1. 检查控制器状态：`rostopic echo /controller/hero_chassis_controller/state`
2. 检查关节是否接收到力：`rostopic echo /joint_states`
3. 调整PID参数

### 问题3：RViz显示异常
**症状**: 模型不显示或TF树错误
**解决**:
1. 确认 use_joint_state_publisher:=true
2. 检查 robot_state_publisher 是否运行
3. 查看 TF 树：`rosrun rqt_tf_tree rqt_tf_tree`

## 📝 日志文件
- 编译日志: `logs/build.log`
- Display日志: `logs/display_with_tf.log`
- Controller日志: `logs/load_controller.log`

## 🎯 预期行为
1. Gazebo窗口显示麦轮底盘机器人
2. RViz显示完整的URDF模型和TF树
3. 发送/cmd_vel命令后机器人响应移动
4. /odom话题发布里程计信息
5. /joint_states话题发布四个轮子的状态
