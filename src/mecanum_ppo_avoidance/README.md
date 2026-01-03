# 麦轮PPO轨迹规划

## 进度安排

- 12月25日：导入麦轮底盘模型，在gazebo和rviz中进行显示
- 12月26日：当前运行 `load_controller.launch` 控制器没有被正确加载出来-> 在 `rosservice call /controller_manager/list_controllers` 命令查看控制器时，没有找到任何被加载的控制器，命令行返回 `controller: []`，我希望应该正确加载 `mecanum_chassis_controller `, 运行日志保存在 `run_with_log.log` 中
## 代办事项
- 12月27日：

- 正确完成 `rviz` 中底盘模型的显示，包括材质和关节TF变换的发布
- 完成底盘 `ros_control` 控制器的编写，使用力矩控制器控制底盘
- 底盘正逆学解算的实现
- 键盘控制节点测试底盘运动学正逆解
- 先测试 `move_base` 路径规划
- 然后给底盘适配PPO路径规划
- 进行单元测试
- 交付

## 问题

### 启动gazebo异常退出
- 原因：
    - Gazebo-2 在控制器启动几秒后总是以代码 139 退出，因此崩溃发生在 gzserver 进程内部，而不是启动基础架构中。

    - 在生成器完成后，gzserver 内部唯一运行的自定义代码是麦克纳姆控制器。在 `mecanum_chassis_controller.cpp`:166-175 中，`effortback()` 订阅者盲目地解引用了 `msg->effort[0..3]` 和 `msg->velocity[0..3]`。
    - 启动时，`/joint_states` 仍然由 GUI 的 `joint_state_publisher` 发布；其消息的 `effort` 数组为空（有时总条目数少于四个）。第一个到达 `effortback()` 的消息在 `gzserver` 的控制器线程中读取到了向量的末尾，导致段错误，最终导致 Gazebo 崩溃。

- 解决方案：
    - 保留订阅器，请对其进行保护：仅当 `effort` 和 `velocity` 都包含至少四个元素时才复制数据，否则立即返回。此外，将数组初始化为零（例如 `std::array<double, 4> joint_torque_{}`），这样在等待第一个有效消息时，限制器逻辑就不会除以垃圾值。

    - 添加保护（或切换到关节句柄反馈）后，重新运行 `roslaunch mecanum_ppo_avoidance load_controller.launch`；控制器应该保持运行，即使其他节点发布不完整的 `JointState` 消息，Gazebo 也不会再崩溃。
