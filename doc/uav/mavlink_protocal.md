# MAVLink协议详解

## 1. MAVLink简介

### 1.1 什么是MAVLink？

**MAVLink**（Micro Air Vehicle Link）是一个轻量级的消息传递协议，专为无人机（UAV）和地面控制站（GCS）之间的通信而设计。它是目前无人机领域使用最广泛的通信协议之一。

### 1.2 主要特点

- **轻量级**：占用带宽小，适合低带宽通信链路
- **高效性**：使用二进制序列化，传输效率高
- **可靠性**：内置校验和机制，确保数据完整性
- **跨平台**：支持C、C++、Python、Java等多种编程语言
- **标准化**：被PX4、ArduPilot等主流飞控系统采用

### 1.3 版本演进

- **MAVLink 1.0**：早期版本，包ID为8位（最多支持256个包）
- **MAVLink 2.0**：当前主流版本，包ID扩展到24位，支持消息签名和扩展字段

## 2. MAVLink协议结构

### 2.1 MAVLink 2.0消息帧格式

```
+--------+--------+--------+--------+--------+--------+--------+--------+
| STX    | LEN    | INC    | CMP    | SEQ    | SYS ID | CMP ID | MSG ID |
| 1 byte | 1 byte | 1 byte | 1 byte | 1 byte | 1 byte | 1 byte | 3 bytes|
+--------+--------+--------+--------+--------+--------+--------+--------+
| PAYLOAD (0-255 bytes)                                                 |
+-----------------------------------------------------------------------+
| CHECKSUM | SIGNATURE (optional, 13 bytes)                             |
| 2 bytes  |                                                             |
+----------+-------------------------------------------------------------+
```

### 2.2 字段说明

| 字段 | 长度 | 说明 |
|------|------|------|
| **STX** | 1字节 | 起始标志，MAVLink 2.0为0xFD |
| **LEN** | 1字节 | 有效载荷长度 |
| **INC** | 1字节 | 不兼容标志（Incompatibility flags） |
| **CMP** | 1字节 | 兼容标志（Compatibility flags） |
| **SEQ** | 1字节 | 消息序列号，用于检测丢包 |
| **SYS ID** | 1字节 | 系统ID（发送者的唯一标识） |
| **CMP ID** | 1字节 | 组件ID（发送者组件的标识） |
| **MSG ID** | 3字节 | 消息ID（消息类型标识） |
| **PAYLOAD** | 0-255字节 | 消息内容 |
| **CHECKSUM** | 2字节 | CRC校验和 |
| **SIGNATURE** | 13字节 | 可选的消息签名（用于安全认证） |

### 2.3 系统ID和组件ID

#### 系统ID (System ID)
- 范围：1-255
- 0表示广播
- 示例：
  - 1: 飞控
  - 255: 地面控制站

#### 组件ID (Component ID)
- 标识系统中的不同组件
- 常见组件ID：
  - 0: 所有组件（广播）
  - 1: 自动驾驶仪（Autopilot）
  - 100: 相机
  - 190: 云台
  - 191: 机载计算机

## 3. MAVLink消息类型

### 3.1 消息分类

#### 3.1.1 心跳消息 (HEARTBEAT)
- **消息ID**: 0
- **用途**: 周期性发送，表明系统存活
- **频率**: 通常1Hz
- **内容**: 系统类型、飞行模式、系统状态等

```cpp
// 心跳消息示例
mavlink_msg_heartbeat_pack(
    system_id,           // 系统ID
    component_id,        // 组件ID
    &msg,                // 消息缓冲区
    MAV_TYPE_QUADROTOR,  // 无人机类型
    MAV_AUTOPILOT_PX4,   // 飞控类型
    base_mode,           // 基本模式
    custom_mode,         // 自定义模式
    system_state         // 系统状态
);
```

#### 3.1.2 系统状态消息
- **SYSTEM_STATUS** (MSG ID: 1): 电池、通信状态等
- **SYS_STATUS** (MSG ID: 1): 传感器健康状态
- **BATTERY_STATUS** (MSG ID: 147): 详细电池信息

#### 3.1.3 位置和导航消息
- **GLOBAL_POSITION_INT** (MSG ID: 33): 全局位置（经纬度）
- **LOCAL_POSITION_NED** (MSG ID: 32): 本地位置（NED坐标系）
- **ATTITUDE** (MSG ID: 30): 姿态角（roll, pitch, yaw）
- **ATTITUDE_QUATERNION** (MSG ID: 31): 四元数表示的姿态

#### 3.1.4 控制命令消息
- **COMMAND_LONG** (MSG ID: 76): 执行命令
- **COMMAND_INT** (MSG ID: 75): 带整数参数的命令
- **SET_POSITION_TARGET_LOCAL_NED** (MSG ID: 84): 设置本地目标位置
- **SET_POSITION_TARGET_GLOBAL_INT** (MSG ID: 86): 设置全球目标位置

#### 3.1.5 任务规划消息
- **MISSION_COUNT** (MSG ID: 44): 任务点总数
- **MISSION_ITEM** (MSG ID: 39): 任务点
- **MISSION_REQUEST** (MSG ID: 40): 请求任务点
- **MISSION_ACK** (MSG ID: 47): 任务确认

### 3.2 消息请求-响应模式

MAVLink采用请求-响应模式：

```
地面站                           无人机
   |                               |
   |---MISSION_REQUEST_LIST------->|
   |                               |
   |<-------MISSION_COUNT----------|
   |                               |
   |---MISSION_REQUEST(0)--------->|
   |                               |
   |<-------MISSION_ITEM(0)--------|
   |                               |
   |---MISSION_ACK---------------->|
```

## 4. MAVLink命令系统

### 4.1 MAV_CMD命令

MAVLink定义了一系列标准命令（MAV_CMD），通过COMMAND_LONG或COMMAND_INT消息发送。

#### 常用命令示例

```cpp
// 起飞命令
MAV_CMD_NAV_TAKEOFF (22)
参数1: 俯仰角
参数2-6: 空
参数7: 目标高度

// 降落命令
MAV_CMD_NAV_LAND (21)
参数1: 中止高度
参数4: 偏航角
参数7: 目标高度

// 切换模式
MAV_CMD_DO_SET_MODE (176)
参数1: 模式
参数2: 自定义模式
参数3: 子模式

// 解锁/上锁电机
MAV_CMD_COMPONENT_ARM_DISARM (400)
参数1: 1=解锁, 0=上锁
```

### 4.2 命令发送示例

```cpp
mavlink_message_t msg;
mavlink_command_long_t cmd;

cmd.target_system = target_system;
cmd.target_component = target_component;
cmd.command = MAV_CMD_NAV_TAKEOFF;
cmd.confirmation = 0;
cmd.param1 = 0;  // 俯仰角
cmd.param2 = 0;
cmd.param3 = 0;
cmd.param4 = 0;
cmd.param5 = 0;
cmd.param6 = 0;
cmd.param7 = 10.0;  // 起飞高度10米

mavlink_msg_command_long_encode(system_id, component_id, &msg, &cmd);
```

## 5. 坐标系统

### 5.1 NED坐标系
- **N (North)**: 北向为X轴正方向
- **E (East)**: 东向为Y轴正方向
- **D (Down)**: 地向为Z轴正方向（与常见的Z轴向上相反）

### 5.2 机体坐标系
- **X轴**: 机头方向
- **Y轴**: 机身右侧
- **Z轴**: 机身向下

### 5.3 全局坐标系
- 使用WGS84标准
- 经度（Longitude）
- 纬度（Latitude）
- 海拔高度（Altitude）

## 6. MAVLink工具和库

### 6.1 代码生成工具

MAVLink使用XML定义消息格式，然后生成各种语言的代码：

```bash
# 安装MAVLink
pip install pymavlink

# 生成C代码
python -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 \
       --output=generated message_definitions/v1.0/common.xml
```

### 6.2 常用工具

- **mavproxy**: 命令行地面控制站
- **QGroundControl**: 图形化地面控制站
- **MAVLink Router**: MAVLink消息路由器
- **mavlink-devguide**: 官方开发文档

### 6.3 Python示例

```python
from pymavlink import mavutil

# 连接到飞控
master = mavutil.mavlink_connection('udp:localhost:14550')

# 等待心跳
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % 
      (master.target_system, master.target_component))

# 发送解锁命令
master.arducopter_arm()

# 接收消息
while True:
    msg = master.recv_match(blocking=True)
    if msg.get_type() == 'ATTITUDE':
        print("Roll: %.2f, Pitch: %.2f, Yaw: %.2f" % 
              (msg.roll, msg.pitch, msg.yaw))
```

## 7. MAVLink实战应用

### 7.1 通信链路配置

常见的通信方式：
- **串口**: `/dev/ttyUSB0`, `/dev/ttyACM0`
- **UDP**: `udp:127.0.0.1:14550`
- **TCP**: `tcp:192.168.1.1:5760`

### 7.2 消息过滤和路由

```python
# 请求特定消息流
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,  # 频率 (Hz)
    1    # 启用
)
```

### 7.3 调试技巧

```bash
# 使用mavproxy监听消息
mavproxy.py --master=/dev/ttyACM0 --baudrate=57600

# 查看特定消息
> watch ATTITUDE

# 发送命令
> arm throttle
> mode GUIDED
> takeoff 10
```

## 8. 安全性考虑

### 8.1 消息签名

MAVLink 2.0支持消息签名，防止未授权访问：

```cpp
// 启用签名
mavlink_setup_signing(
    &signing,           // 签名结构
    secret_key,         // 密钥
    timestamp           // 时间戳
);
```

### 8.2 通信加密

虽然MAVLink本身不提供加密，但可以：
- 使用VPN隧道
- 配置SSL/TLS加密
- 使用专用的加密数据链

## 9. 性能优化

### 9.1 消息频率控制

不同消息类型的推荐频率：
- 心跳: 1 Hz
- 姿态: 10-50 Hz
- 位置: 1-10 Hz
- GPS: 1-5 Hz

### 9.2 带宽管理

```cpp
// 根据链路质量动态调整消息率
if (link_quality < 50) {
    reduce_message_rate();
}
```

## 10. 学习资源

### 10.1 官方文档
- [MAVLink官方网站](https://mavlink.io/)
- [消息定义](https://mavlink.io/en/messages/common.html)
- [开发者指南](https://mavlink.io/en/guide/mavlink_2.html)

### 10.2 开源项目
- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot)
- [ArduPilot](https://github.com/ArduPilot/ardupilot)
- [QGroundControl](https://github.com/mavlink/qgroundcontrol)

### 10.3 社区资源
- [PX4论坛](https://discuss.px4.io/)
- [ArduPilot论坛](https://discuss.ardupilot.org/)

## 11. 常见问题

### Q1: MAVLink 1.0和2.0可以互通吗？
A: 可以，MAVLink 2.0向下兼容1.0，但需要正确配置。

### Q2: 如何处理消息丢失？
A: 使用SEQ字段检测丢包，关键消息需要实现重传机制。

### Q3: 如何自定义MAVLink消息？
A: 修改XML定义文件，重新生成代码。建议使用自定义消息ID范围。

### Q4: 最大有效载荷是多少？
A: MAVLink 2.0最大255字节。

---

**最后更新**: 2026年1月2日
