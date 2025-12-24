#### 开源自主导航小车MickX4

- [1 ROS底盘运动控制](#1-ros底盘运动控制)
  - [1.1 小车控制流程图](#11-小车控制流程图)
  - [1.2 小车控制核心代码分析](#12-小车控制核心代码分析)
    - [1.2.1 小车核心代码-遥控器解析](#121-小车核心代码-遥控器解析)
    - [1.2.2 小车核心代码-电机反馈数据](#122-小车核心代码-电机反馈数据)
    - [1.2.3 小车核心代码-PID控制](#123-小车核心代码-pid控制)
    - [1.2.3 小车核心代码-上位机通讯接口](#123-小车核心代码-上位机通讯接口)
  - [参考资料](#参考资料)

## 1 ROS底盘运动控制

小车的底盘控制器我们使用的是STM32F103ZET6型号的单片机（代码兼容STM32F103的型号）。单片机主要负责接收上位机和遥控器的指令，采集电机的实时状态数据，实现电机的速度闭环控制。使用单片机的好处在于单片机实时性强，IO端口丰富，对底层硬件设备友好。

这里我们使用了多个电路板分层的思想去设计底盘的控制器，主要分为 通讯板、电源板、最小系统、传感器接口，这种结构的优势在于：快速开发、积木式组合、故障快速更换。

这里主要使用到了电源板提供 19V 12V 5V 电源，通讯板提供232接口，CAN和IIC接口,添加了一个MPU6050和5883L (后面换成了MPU9250)

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/1ff1f86cae203c8deb8e19d100f2dfb1.png#pic_center)

### 1.1 小车控制流程图

下图展示了小车底盘运动控制的功能模块图，单片机控制器主要包含有4个功能：1）解析遥控器和上位机发送过来的数据指令。2）分别设置一个PID控制器控制每一个轮子的速度。3）解析电机反馈的电机状态数据。4）上传电机数据到上位机。

这里涉及到的STM32的模块有：**CAN模块**，**串口2中断接收**、**串口1DMA接收**、**IIC模块**  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/98a529f1e1b8ccdc031d178b6770f866.png#pic_center)  
我们在实现上述功能的时候，将实时性要求高的操作，如PID运算，电机状态数据读取等操作放在了中断函数中，而读取遥控器数据设置小车目标速度、上传电机状态数据等实时性要求相对较弱的操作放在主循环中，通过状态变量实现。

小车底盘完整代码见github上[【2】](https://github.com/RuPingCen/mick_robot_chasiss) （https://github.com/RuPingCen/mick\_robot\_chasiss）

小车速度控制采用的是位置型PI控制器  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/9ad583422bdda3a12569f6288d081d16.png#pic_center)  
（图片来源于：\[4\]）  
Kp:比例系数 Ki积分系数 Kd 微分系数（我们的代码中设置为0）

### 1.2 小车控制核心代码分析

#### 1.2.1 小车核心代码-遥控器解析

我们所使用的[大疆DT6遥控器](https://www.robomaster.com/zh-CN/products/components/detail/122)（参加Robmaster所用的设备）和配套的DR16接收机输出的信号为标准的DBUS协议数据，接收机每隔14ms通过DBUS发送一帧18字节数据。这里我们使用串口1-DMA模式接收，并解析大疆遥控器发送过来的指令  
遥控器发送的前6个字节（48个bit）为遥控器的四个通道的摇杆值和两个拨动开关值。

bit0-bit10 通道0的值，bit11-bit21为通道1，bi22-bit32 通道2的值，bit33-bit43为通道3 ，bit44-bit45为开关S1的值， bit46-bit47为开关S2的值。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/048bb5b6baec30cca4ec2ddcfadddf12.png#pic_center)

解码的程序可参考如下代码段：（dbus\_rc为自定义结构体，该解析代码参考了大疆官方的demo）

```c
void RC_Callback_Handler(uint8_t *pData)
{
	if(pData == NULL)
	{
		return;
	}
	
	dbus_rc.ch1 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	dbus_rc.ch2 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5))	& 0x07FF;
	dbus_rc.ch3 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
	dbus_rc.ch4 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) &0x07FF;
	dbus_rc.sw1 = ((pData[5] >> 4) & 0x000C) >> 2;
	dbus_rc.sw2 = ((pData[5] >> 4) & 0x0003);
	dbus_rc.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	dbus_rc.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	dbus_rc.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
	dbus_rc.press_l = pData[12];
	dbus_rc.press_r = pData[13];
	dbus_rc.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
	
	if(DBUS_flag == DBUS_INIT) 
	{
		dbus_rc.available =0x00;
		if(RC_Offset_Init())
			DBUS_flag = DBUS_RUN;
		else
			DBUS_flag = DBUS_INIT;
	}
	else
	{
		if((dbus_rc.ch1 > 2000) || (dbus_rc.ch1<100) 
			|| (dbus_rc.ch3 > 2000) || (dbus_rc.ch3<100)
		  || (dbus_rc.sw1 > 3) || (dbus_rc.sw1<1)
		  || (dbus_rc.sw2 > 3) || (dbus_rc.sw2<1))
		{
			dbus_rc.available =0x00;
		}
		else
		{
			dbus_rc.available =0x01;
		}
		dbus_rc.cnt =dbus_rc.v +1;	
	}
}
```

这里我们增加了一些校验措施，防止由于连线松动带来的乱码的现象。

#### 1.2.2 小车核心代码-电机反馈数据

电机的解析函数是放在CAN总线的中断里面的，当CAN总线上有发送数据给单片机时（标识符等于单片机设置的过滤标识符时触发中断），stm32自动触发CAN中断接受数据，并写入到对应的结构体上。

我们使用的大疆M3508电机反馈的数据中包含了电机的转角、速度、电流、温度信息，因此我们定义了一个如下的结构体用于保存电机的数据  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/13a327b1bcffffdc43985b05f100acbd.png#pic_center)  
数据接收结构体

```c
typedef struct{
		uint16_t 	angle;				//abs angle range:[0,8191] 电机转角绝对值
		uint16_t 	last_angle;	  //abs angle range:[0,8191]
		int16_t	 	speed_rpm;       //转速
		int16_t  	real_current;    //转速
		int16_t  	given_current;   //实际的转矩电流
		uint8_t  	Temp;           //温度
		uint16_t	offset_angle;   //电机启动时候的零偏角度
		int32_t		round_cnt;     //电机转动圈数
		int32_t		total_angle;    //电机转动的总角度
		uint8_t		buf_idx;
		uint16_t	angle_buf[FILTER_BUF_LEN];
		uint16_t	fited_angle;
		uint32_t	msg_cnt;
}moto_measure_t;
```

每产生一次CAM中断，将CAN buffer中的消息写入到电机数据结构体中

```c
void get_moto_measure(moto_measure_t *ptr,CanRxMsg* RxMessage)
{
	int delta=0;
	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RxMessage->Data[0]<<8 | RxMessage->Data[1]) ; //电机转角
	ptr->real_current  = (int16_t)(RxMessage->Data[2]<<8 | RxMessage->Data[3]);
	ptr->speed_rpm = ptr->real_current;	
	
	ptr->given_current = (int16_t)(RxMessage->Data[4]<<8 | RxMessage->Data[5])/-5;
	ptr->Temp = RxMessage->Data[6];
	
	if(ptr->speed_rpm > 10 ) //电机正转
	{
		if((ptr->angle - ptr->last_angle) >= 50)
		{
		  delta = ptr->angle - ptr->last_angle;
		}
		else if ((ptr->angle - ptr->last_angle) <= -50)
		{
		  delta = ptr->angle + 8192 - ptr->last_angle;
			ptr->round_cnt++;
		}
		else;
	}
  else if(ptr->speed_rpm < -10) //电机反转
	{
		if ((ptr->angle - ptr->last_angle) <= -50)
		{
			delta =ptr->angle - ptr->last_angle;
		}
		else if((ptr->angle - ptr->last_angle) >= 50)
		{
			delta = ptr->angle - 8192 - ptr->last_angle;
			ptr->round_cnt--;
		}
		else;
	}
	else
		delta=0;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}
```

#### 1.2.3 小车核心代码-PID控制

小车的速度经过运动学模型分解到每一个轮子的转速后，对每个轮子都设置一个PID控制器。PID运行的频率在1KHZ，PID在定时器中断中调用，保证控制周期固定。

```c
float pid_sp_calc(pid_t* pid, float get, float set, float gyro)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    pid->pout = pid->p * pid->err[NOW];
    
	if(fabs(pid->i) >= 0.001f)
		pid->iout += pid->i * pid->err[NOW];
	else
		pid->iout = 0;
	pid->dout = -pid->d * gyro/100.0f;	
	abs_limit(&(pid->iout), pid->IntegralLimit);
	pid->pos_out = pid->pout + pid->iout + pid->dout;
	abs_limit(&(pid->pos_out), pid->MaxOutput);
	pid->last_pos_out = pid->pos_out;	//update last time 

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
}
```

#### 1.2.3 小车核心代码-上位机通讯接口

小车需要与上位机进行通讯，以上传控制器状态和电机数据，同时我们还需要让小车接收上位机下发的速度v和角速度w命令，实现小车在预定的速度参数下运行。

底盘接口需要根据具体需求进行定义，通常小车底盘应当具备以下功能

1、接收上位机下发的速度和角速度指令

2、接收上位机下发的IO控制指令，控制车辆的转向灯以及其他的IO设备

3、上传每一个电机的电流、速度、位置信息

4、上传超声波传感器的数据

5、上传底盘IMU的数据及姿态信息

6、上传底盘状态数据，如电池电量，程序错误代码

下图是我们所使用的底盘与上位机的一个接口协议[【3】](https://docs.qq.com/sheet/DV2hmSEdSYVVtclB4)。（也就是和上位机约定一个格式，表明一段数据中第1个字节表示帧头，第5个字节表示速度，第6个字节表示位置）

![上位机帧协议
](https://i-blog.csdnimg.cn/blog_migrate/4b3af7657ac1ede2d3eec11548b6ed51.png#pic_center)

这里我们对底盘提供了4个下发接口，和5个上传接口，具体可以按照实际的项目进行增加和删减。 上位机可通过转速命令（单位RPM）、速度命令设置小车的速度。底盘上传的电机的转速和转角数据主要用于后续计算里程计使用。

小车底盘完整代码放在了github上[【2】](https://github.com/RuPingCen/mick-robot-chasiss)，通讯协议[【3】](https://docs.qq.com/sheet/DV2hmSEdSYVVtclB4)

### 参考资料

【1】https://www.robomaster.com/zh-CN/products/components/detail/122  
【2】当前小车底盘的代码位于：https://github.com/RuPingCen/mick\_robot\_chasiss  
【3】ROS底盘数据帧协议v1.1 https://docs.qq.com/sheet/DV2hmSEdSYVVtclB4  
【4】https://zhuanlan.zhihu.com/p/234740503?utm\_source=wechat\_session&utm\_medium=social&s\_r=0#showWechatShareTip

上一篇：[开源自主导航小车MickX4（一）ROS底盘硬件](https://blog.csdn.net/crp997576280/article/details/108290182) 下一篇：[开源自主导航小车MickX4（三）底盘ROS节点](https://blog.csdn.net/crp997576280/article/details/108567732)

博客每周一更新，欢迎大家关注收藏。

**欢迎大家点赞在评论区交流讨论（cenruping@vip.qq.com） O(∩\_∩)O**

或者加群交流（1149897304）  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/e7e3b9b172150d7e0831c2b41f147adc.png#pic_center)