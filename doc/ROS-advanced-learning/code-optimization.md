# ROS代码优化与最佳实践

本文档总结ROS开发中的代码优化技巧、常见错误处理方法和最佳实践。

## 目录

- [代码优化技巧](#代码优化技巧)
  - [循环注册Handle](#循环注册handle)
  - [主循环回调化](#主循环回调化)
- [异常和错误处理](#异常和错误处理)
  - [空指针检查](#空指针检查)
  - [容器插入前查重](#容器插入前查重)
  - [资源管理异常](#资源管理异常)
- [常见编译错误](#常见编译错误)
  - [链接错误：undefined reference](#链接错误undefined-reference)
  - [CMakeLists.txt配置问题](#cmakeliststxt配置问题)
- [运行时错误排查](#运行时错误排查)
  - [ros::ok()返回false](#rosok返回false)
  - [频率不匹配问题](#频率不匹配问题)
- [开发环境问题](#开发环境问题)
  - [CLion与终端冲突](#clion与终端冲突)

---

## 代码优化技巧

### 循环注册Handle

#### 问题描述

在ROS控制器中注册关节接口时，手动逐个注册关节会导致代码冗长、难以维护。

#### 原始写法（不推荐）

```cpp
// 硬编码每个关节名称，扩展性差
effort_joint_handles_.push_back(effort_joint_interface->getHandle("joint1"));
effort_joint_handles_.push_back(effort_joint_interface->getHandle("joint2"));
effort_joint_handles_.push_back(effort_joint_interface->getHandle("joint3"));
// ... 需要为每个关节重复代码
```

**缺点：**
- 添加新关节需要修改代码
- 容易出现拼写错误
- 代码冗余度高

#### 优化写法（推荐）

```cpp
// 使用循环自动注册所有关节
auto effort_joint_interface = this->get<hardware_interface::EffortJointInterface>();
std::vector<std::string> names = effort_joint_interface->getNames();
for (const auto& name : names)
{
  effort_joint_handles_.push_back(effort_joint_interface->getHandle(name));
}
```

**优点：**
- 自动适应关节数量变化
- 代码简洁易维护
- 减少人为错误

#### 进阶：带异常处理的注册

```cpp
auto effort_joint_interface = this->get<hardware_interface::EffortJointInterface>();
std::vector<std::string> names = effort_joint_interface->getNames();

for (const auto& name : names)
{
  try
  {
    effort_joint_handles_.push_back(effort_joint_interface->getHandle(name));
    ROS_INFO_STREAM("Successfully registered joint: " << name);
  }
  catch (const hardware_interface::HardwareInterfaceException& ex)
  {
    ROS_ERROR_STREAM("Failed to register joint '" << name << "': " << ex.what());
    return false;
  }
}
```

#### 实际应用场景

在 `rm_control` 或自定义硬件接口中批量注册关节：

```cpp
bool MyController::init(hardware_interface::RobotHW* robot_hw,
                        ros::NodeHandle& controller_nh)
{
  // 获取关节名称列表（从参数服务器）
  if (!controller_nh.getParam("joints", joint_names_))
  {
    ROS_ERROR("No joints specified");
    return false;
  }

  // 批量注册
  auto* effort_iface = robot_hw->get<hardware_interface::EffortJointInterface>();
  for (const auto& joint_name : joint_names_)
  {
    try
    {
      joint_handles_.push_back(effort_iface->getHandle(joint_name));
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("Could not find joint '" << joint_name << "'");
      return false;
    }
  }
  
  return true;
}
```

---

### 主循环回调化

#### 问题描述

传统的ROS节点使用 `while(ros::ok())` 主循环，需要手动管理循环频率和回调处理，代码复杂且容易出错。

#### 传统写法（不推荐）

```cpp
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_hardware_interface");
  ros::NodeHandle nh;
  
  ROBOTHardwareInterface robot(nh);
  ros::Rate loop_rate(50);  // 50Hz
  
  while (ros::ok())
  {
    // 手动调用更新
    robot.read();
    robot.update();
    robot.write();
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
```

**缺点：**
- 需要手动管理循环频率
- `spinOnce()` 只处理一个回调，可能阻塞
- 不支持多线程回调
- 复杂的定时逻辑容易出错

#### 优化写法：使用定时器（推荐）

```cpp
class ROBOTHardwareInterface
{
public:
  ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh)
  {
    // 使用定时器替代主循环
    double frequency = 50.0;  // 50Hz
    control_timer_ = nh_.createTimer(
        ros::Duration(1.0 / frequency),
        &ROBOTHardwareInterface::update,
        this);
  }

private:
  void update(const ros::TimerEvent& event)
  {
    // 自动以指定频率调用
    read();
    controller_manager_->update(ros::Time::now(), event.current_real - event.last_real);
    write();
  }

  ros::NodeHandle nh_;
  ros::Timer control_timer_;
  std::unique_ptr<controller_manager::ControllerManager> controller_manager_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_hardware_interface");
  ros::NodeHandle nh;
  
  ROBOTHardwareInterface robot(nh);
  
  // 使用多线程Spinner
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  
  return 0;
}
```

**优点：**
- 定时器自动管理频率
- 支持多线程处理回调
- 代码结构清晰
- 更好的实时性能

#### Spinner类型对比

| Spinner类型 | 线程数 | 使用场景 |
|------------|--------|----------|
| `ros::spin()` | 1 | 单线程，简单节点 |
| `ros::spinOnce()` | 0（手动） | 需要自定义循环控制 |
| `ros::AsyncSpinner` | N | 后台异步处理，主线程继续执行 |
| `ros::MultiThreadedSpinner` | N | 并发处理多个回调，高吞吐量 |

#### AsyncSpinner示例

```cpp
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_hardware_interface");
  ros::NodeHandle nh;
  
  ROBOTHardwareInterface robot(nh);
  
  // 启动4个线程处理回调
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  // 主线程可以继续做其他事情
  ros::waitForShutdown();
  
  return 0;
}
```

#### 实际应用：rm_hw硬件接口

```cpp
class RmRobotHWLoop
{
public:
  RmRobotHWLoop(ros::NodeHandle& nh, std::shared_ptr<RmRobotHW> hardware_interface)
    : nh_(nh), hardware_interface_(hardware_interface)
  {
    // 创建控制器管理器
    controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
        hardware_interface_.get(), nh_);
    
    // 从参数服务器获取控制频率
    double loop_hz;
    nh_.param("loop_hz", loop_hz, 100.0);
    
    // 创建控制循环定时器
    loop_timer_ = nh_.createTimer(
        ros::Duration(1.0 / loop_hz),
        &RmRobotHWLoop::update,
        this);
  }

private:
  void update(const ros::TimerEvent& e)
  {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    hardware_interface_->read(ros::Time::now(), elapsed_time_);
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    hardware_interface_->write(ros::Time::now(), elapsed_time_);
  }

  ros::NodeHandle nh_;
  ros::Timer loop_timer_;
  ros::Duration elapsed_time_;
  std::shared_ptr<RmRobotHW> hardware_interface_;
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};
```

---

## 异常和错误处理

### 空指针检查

#### 问题描述

在ROS控制系统中，许多函数返回指针或句柄，使用前必须检查有效性，否则会导致段错误。

#### 基础检查

```cpp
// 检查buffer指针
if (!buffer)
{
  throw hardware_interface::HardwareInterfaceException(
      "Cannot create handle '" + name + "'. Tf Buffer data pointer is null.");
}
```

#### 完整的资源检查模式

```cpp
bool MyController::init(hardware_interface::RobotHW* robot_hw,
                        ros::NodeHandle& controller_nh)
{
  // 1. 检查硬件接口指针
  if (!robot_hw)
  {
    ROS_ERROR("RobotHW pointer is null");
    return false;
  }

  // 2. 获取接口并检查
  auto* effort_iface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (!effort_iface)
  {
    ROS_ERROR("This controller requires an EffortJointInterface");
    return false;
  }

  // 3. 获取参数并检查
  std::string joint_name;
  if (!controller_nh.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint name specified");
    return false;
  }

  // 4. 获取句柄并捕获异常
  try
  {
    joint_handle_ = effort_iface->getHandle(joint_name);
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("Failed to get joint handle: " << e.what());
    return false;
  }

  return true;
}
```

#### TF Buffer检查示例

```cpp
#include <tf2_ros/buffer.h>

class MyNode
{
public:
  MyNode(ros::NodeHandle& nh)
  {
    // 创建TF buffer
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    
    if (!tf_buffer_)
    {
      throw std::runtime_error("Failed to create TF buffer");
    }
    
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  bool getTransform(const std::string& target, const std::string& source)
  {
    try
    {
      auto transform = tf_buffer_->lookupTransform(target, source, ros::Time(0));
      return true;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_STREAM("TF lookup failed: " << ex.what());
      return false;
    }
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
```

---

### 容器插入前查重

#### 问题描述

向容器（如 `std::map`、`std::unordered_map`）插入元素时，如果键已存在，直接覆盖可能导致资源泄漏或逻辑错误。

#### 标准模式

来自 `hardware_interface/internal/interface_manager.h`：

```cpp
// 检查接口是否已注册
if (interfaces_.find(iface_name) != interfaces_.end())
{
  ROS_WARN_STREAM("Replacing previously registered interface '" << iface_name << "'.");
}

// 注册新接口
interfaces_[iface_name] = iface;
internal::CheckIsResourceManager<T>::callGetResources(resources_[iface_name], iface);
```

#### 实际应用：注册硬件接口

```cpp
template <class T>
void InterfaceManager::registerInterface(T* iface)
{
  const std::string iface_name = internal::demangledTypeName<T>();
  
  // 查重检查
  if (interfaces_.find(iface_name) != interfaces_.end())
  {
    ROS_WARN_STREAM("Interface '" << iface_name << "' is already registered. "
                    << "Replacing with new instance.");
  }
  
  // 存储接口
  interfaces_[iface_name] = iface;
  
  // 提取资源列表
  internal::CheckIsResourceManager<T>::callGetResources(resources_[iface_name], iface);
  
  ROS_DEBUG_STREAM("Registered interface: " << iface_name);
}
```

#### 更安全的查重插入模式

```cpp
// 方法1：使用insert()返回值
auto result = my_map.insert({key, value});
if (!result.second)
{
  ROS_WARN_STREAM("Key '" << key << "' already exists, not inserting.");
}

// 方法2：使用emplace()
if (!my_map.emplace(key, value).second)
{
  ROS_ERROR_STREAM("Failed to insert key '" << key << "'");
}

// 方法3：先检查后插入
if (my_map.count(key) > 0)
{
  // 键已存在，决定是覆盖还是跳过
  ROS_WARN_STREAM("Key exists, overwriting...");
}
my_map[key] = value;
```

#### 控制器注册查重

```cpp
class ControllerManager
{
public:
  bool loadController(const std::string& name)
  {
    // 检查是否已加载
    if (controllers_.find(name) != controllers_.end())
    {
      ROS_ERROR_STREAM("Controller '" << name << "' is already loaded");
      return false;
    }
    
    // 创建控制器
    auto controller = controller_loader_->createInstance(name);
    if (!controller)
    {
      ROS_ERROR_STREAM("Failed to create controller '" << name << "'");
      return false;
    }
    
    // 插入map
    controllers_[name] = controller;
    ROS_INFO_STREAM("Loaded controller: " << name);
    return true;
  }

private:
  std::map<std::string, ControllerBasePtr> controllers_;
  std::shared_ptr<pluginlib::ClassLoader<ControllerBase>> controller_loader_;
};
```

---

### 资源管理异常

#### RAII模式

Resource Acquisition Is Initialization（资源获取即初始化）是C++中管理资源的最佳实践。

```cpp
class HardwareInterfaceGuard
{
public:
  HardwareInterfaceGuard(RobotHW* hw) : hw_(hw)
  {
    if (!hw_)
    {
      throw std::invalid_argument("Hardware interface pointer is null");
    }
    
    // 初始化硬件
    hw_->init();
    ROS_INFO("Hardware initialized");
  }
  
  ~HardwareInterfaceGuard()
  {
    // 自动清理资源
    if (hw_)
    {
      hw_->shutdown();
      ROS_INFO("Hardware shutdown");
    }
  }
  
  // 禁止拷贝
  HardwareInterfaceGuard(const HardwareInterfaceGuard&) = delete;
  HardwareInterfaceGuard& operator=(const HardwareInterfaceGuard&) = delete;

private:
  RobotHW* hw_;
};
```

#### 智能指针管理

```cpp
class ControllerNode
{
public:
  ControllerNode(ros::NodeHandle& nh)
  {
    // 使用智能指针自动管理内存
    robot_hw_ = std::make_shared<MyRobotHW>();
    controller_manager_ = std::make_unique<controller_manager::ControllerManager>(
        robot_hw_.get(), nh);
    
    // 如果构造函数抛异常，智能指针会自动清理已分配的内存
  }
  
  // 析构时自动释放资源
  ~ControllerNode() = default;

private:
  std::shared_ptr<MyRobotHW> robot_hw_;
  std::unique_ptr<controller_manager::ControllerManager> controller_manager_;
};
```

---

## 常见编译错误

### 链接错误：undefined reference

#### 错误示例

```
/usr/bin/ld: CMakeFiles/st_hardware.dir/src/common/hardware_interface.cpp.o: 
  in function `steering_engine_hw::StRobotHW::loadUrdf(ros::NodeHandle&)':
hardware_interface.cpp:(.text+0x11d6): undefined reference to 
  `urdf::Model::initString(std::__cxx11::basic_string<char, std::char_traits<char>, 
   std::allocator<char> > const&)'
collect2: error: ld returned 1 exit status
```

#### 原因分析

这是典型的**链接错误**，表示：
1. 代码中使用了 `urdf::Model::initString()` 函数
2. 编译器找到了函数声明（头文件）
3. 但链接器找不到函数的实现（库文件）

#### 解决方案

**在 `CMakeLists.txt` 中添加缺失的库：**

```cmake
# 找到urdf包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  urdf              # 添加urdf
)

# 链接库
target_link_libraries(st_hardware
  ${catkin_LIBRARIES}
)
```

#### 常见缺失库对应关系

| 错误关键词 | 缺失的包 | CMakeLists添加 |
|-----------|---------|---------------|
| `urdf::Model` | urdf | `urdf` |
| `tf2::Buffer` | tf2_ros | `tf2_ros` |
| `kdl::Tree` | kdl_parser | `kdl_parser orocos_kdl` |
| `sensor_msgs::JointState` | sensor_msgs | `sensor_msgs` |
| `geometry_msgs::Twist` | geometry_msgs | `geometry_msgs` |
| `cv::Mat` | OpenCV | `cv_bridge` |

#### 完整的CMakeLists.txt示例

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_hardware_interface)

## 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  transmission_interface
  urdf
  angles
  pluginlib
  realtime_tools
)

## 声明catkin包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    hardware_interface
    controller_manager
    urdf
  DEPENDS
)

## 指定头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 编译库
add_library(${PROJECT_NAME}
  src/hardware_interface.cpp
  src/robot_hw.cpp
)

## 链接库
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## 编译可执行文件
add_executable(${PROJECT_NAME}_node
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## 安装规则
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
```

---

### CMakeLists.txt配置问题

#### 常见问题1：包含路径错误

```cmake
# 错误：缺少include目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 正确：添加项目自己的include
include_directories(
  include                    # 本项目头文件
  ${catkin_INCLUDE_DIRS}     # ROS包头文件
)
```

#### 常见问题2：依赖顺序错误

```cmake
# 错误：find_package在catkin_package之后
catkin_package()
find_package(catkin REQUIRED COMPONENTS roscpp)

# 正确：先find_package
find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package()
```

#### 常见问题3：缺少导出依赖

```cmake
# package.xml中
<build_depend>urdf</build_depend>
<build_export_depend>urdf</build_export_depend>
<exec_depend>urdf</exec_depend>

# CMakeLists.txt中
catkin_package(
  CATKIN_DEPENDS urdf  # 必须导出
)
```

#### 调试技巧

```bash
# 查看编译详细信息
catkin_make VERBOSE=1

# 清理后重新编译
catkin clean
catkin_make

# 检查包依赖
rospack depends1 your_package_name
rospack depends your_package_name  # 递归依赖
```

---

## 运行时错误排查

### ros::ok()返回false

#### 问题描述

程序运行一段时间后突然停止，调试发现 `while(ros::ok())` 循环退出。

```cpp
while (ros::ok())  // 这里返回false，循环退出
{
  robot.read();
  robot.write();
  ros::spinOnce();
  loop_rate.sleep();
}
```

#### 可能原因

1. **收到SIGINT信号**（Ctrl+C）
2. **节点被远程关闭**（`rosnode kill`）
3. **ROS Master断开连接**
4. **调用了 `ros::shutdown()`**
5. **频率不匹配导致超时**

---

### 频率不匹配问题

#### 问题描述

```cpp
ros::Rate loop_rate(50);  // 设置50Hz

while (ros::ok())
{
  robot.read();   // 假设耗时10ms
  robot.write();  // 假设耗时15ms
  
  ros::spinOnce();
  loop_rate.sleep();  // 期望sleep 20ms，但实际已超时
}
```

**问题：** `read()` + `write()` 耗时25ms，超过了50Hz的周期（20ms），导致实际运行频率低于预期。

#### 解决方案1：调整频率

```cpp
// 测量实际执行时间
ros::Time start = ros::Time::now();
robot.read();
robot.write();
ros::Duration actual_duration = ros::Time::now() - start;

ROS_INFO_STREAM("Actual execution time: " << actual_duration.toSec() * 1000 << "ms");

// 根据实际时间调整频率
// 如果执行时间为25ms，设置频率为30Hz（33ms周期）
ros::Rate loop_rate(30);
```

#### 解决方案2：监控频率

```cpp
#include <ros/duration.h>

ros::Time last_time = ros::Time::now();
ros::Rate loop_rate(50);
int loop_count = 0;

while (ros::ok())
{
  ros::Time current_time = ros::Time::now();
  ros::Duration period = current_time - last_time;
  
  // 每100次循环检查一次频率
  if (++loop_count % 100 == 0)
  {
    double actual_freq = 1.0 / period.toSec();
    ROS_INFO_STREAM("Actual frequency: " << actual_freq << " Hz");
    
    if (actual_freq < 45.0)  // 低于预期5Hz
    {
      ROS_WARN("Control loop running slower than expected!");
    }
  }
  
  last_time = current_time;
  
  robot.read();
  robot.write();
  
  ros::spinOnce();
  loop_rate.sleep();
}
```

#### 解决方案3：使用定时器

```cpp
class RobotController
{
public:
  RobotController(ros::NodeHandle& nh, double frequency)
  {
    // 定时器会自动处理频率问题
    timer_ = nh.createTimer(
        ros::Duration(1.0 / frequency),
        &RobotController::update,
        this);
  }

private:
  void update(const ros::TimerEvent& event)
  {
    // 检测执行时间
    if ((event.current_real - event.last_real).toSec() > (1.0 / 50.0) * 1.1)
    {
      ROS_WARN_STREAM("Control loop duration exceeded: "
                      << (event.current_real - event.last_real).toSec() * 1000 << "ms");
    }
    
    robot_.read();
    robot_.write();
  }

  ros::Timer timer_;
  RobotHW robot_;
};
```

#### 最佳实践

```cpp
#include <std_msgs/Float64.h>

class PerformanceMonitor
{
public:
  PerformanceMonitor(ros::NodeHandle& nh, double target_frequency)
    : target_frequency_(target_frequency)
  {
    // 发布实际频率
    freq_pub_ = nh.advertise<std_msgs::Float64>("control_frequency", 10);
  }

  void startCycle()
  {
    cycle_start_time_ = ros::Time::now();
  }

  void endCycle()
  {
    ros::Duration cycle_duration = ros::Time::now() - cycle_start_time_;
    double actual_frequency = 1.0 / cycle_duration.toSec();
    
    // 发布频率用于监控
    std_msgs::Float64 freq_msg;
    freq_msg.data = actual_frequency;
    freq_pub_.publish(freq_msg);
    
    // 警告
    if (actual_frequency < target_frequency_ * 0.9)
    {
      ROS_WARN_THROTTLE(1.0, "Control frequency dropped to %.1f Hz (target: %.1f Hz)",
                        actual_frequency, target_frequency_);
    }
  }

private:
  double target_frequency_;
  ros::Time cycle_start_time_;
  ros::Publisher freq_pub_;
};

// 使用示例
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_control");
  ros::NodeHandle nh;
  
  RobotHW robot;
  PerformanceMonitor monitor(nh, 50.0);
  ros::Rate loop_rate(50);
  
  while (ros::ok())
  {
    monitor.startCycle();
    
    robot.read();
    robot.write();
    
    monitor.endCycle();
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
```

---

## 开发环境问题

### CLion与终端冲突

#### 问题描述

在CLion中编译一次后，再修改代码在终端编译，修改的代码不会生效，仍然运行的是旧版本。

#### 原因分析

CLion和终端使用不同的构建目录：
- CLion默认使用：`cmake-build-debug/` 或 `cmake-build-release/`
- catkin_make使用：`build/` 和 `devel/`

两者的构建产物相互独立，可能导致混淆。

#### 解决方案

**方法1：清理构建缓存**

```bash
# 清理catkin工作空间
cd ~/catkin_ws
catkin clean

# 重新编译
catkin_make

# 或使用catkin build
catkin clean
catkin build
```

**方法2：清理CLion缓存**

```bash
# 删除CLion构建目录
rm -rf cmake-build-*

# 在CLion中：Tools -> CMake -> Reset Cache and Reload Project
```

**方法3：统一构建工具**

在CLion中配置使用catkin工具：

```
# CLion设置
File -> Settings -> Build, Execution, Deployment -> CMake

# 添加Profile
Name: catkin
Build directory: build
CMake options: -DCATKIN_DEVEL_PREFIX=../devel
```

#### 最佳实践

```bash
# 方案1：只用命令行编译
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 方案2：只用CLion编译
# 在CLion中直接Build -> Build Project

# 方案3：切换工具前清理
catkin clean   # 切换前清理
```

#### 调试技巧

```bash
# 检查当前使用的可执行文件
which your_node_name

# 查看文件修改时间
ls -lh devel/lib/your_package/

# 确认正在运行的程序路径
ps aux | grep your_node_name
```

---

## 代码审查清单

在提交代码前，检查以下项目：

### 编译相关
- [ ] `CMakeLists.txt` 包含所有必要的依赖包
- [ ] `package.xml` 声明了所有运行时依赖
- [ ] 头文件路径正确配置
- [ ] 库链接完整无遗漏

### 异常处理
- [ ] 所有指针使用前检查了非空
- [ ] 关键函数返回值被检查
- [ ] 异常被正确捕获和处理
- [ ] 错误信息具有描述性

### 资源管理
- [ ] 使用智能指针管理动态内存
- [ ] RAII模式正确应用
- [ ] 没有内存泄漏
- [ ] 文件/网络句柄正确关闭

### 性能优化
- [ ] 循环中避免重复计算
- [ ] 使用引用传递大对象
- [ ] 容器预分配空间（`reserve()`）
- [ ] 避免不必要的拷贝

### 代码风格
- [ ] 遵循ROS C++ Style Guide
- [ ] 变量命名清晰明确
- [ ] 函数功能单一明确
- [ ] 注释清楚描述意图

---

## 参考资源

- [ROS Best Practices](http://wiki.ros.org/BestPractices)
- [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)
- [hardware_interface API](http://docs.ros.org/en/api/hardware_interface/html/)
- [controller_manager API](http://docs.ros.org/en/api/controller_manager/html/)
- [rm_control框架文档](https://github.com/rm-controls/rm_control)
