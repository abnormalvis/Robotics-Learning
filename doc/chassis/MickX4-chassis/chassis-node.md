#### 开源自主导航小车MickX4（三）底盘ROS节点

- [1、底盘ROS节点](#1底盘ros节点)
  - [1.1 ROS节点功能定义](#11-ros节点功能定义)
  - [1.2 接收cmd\_vel话题数据](#12-接收cmd_vel话题数据)
  - [1.3 发布里程计数据](#13-发布里程计数据)
  - [1.4 发布IMU数据](#14-发布imu数据)
  - [1.5 发布超声波数据](#15-发布超声波数据)
  - [1.6 TF坐标系](#16-tf坐标系)
- [参考资料](#参考资料)

底盘的ROS节点相当于是对底层硬件封装了一个接口，让软件开发人员不必去关心硬件的实现。开发算法的人员只需要向指定的接口（topic）发送数据即可，底盘硬件开发人员分工合作，这也体现机器人模块化的思想在其中。

ROS节点负责与底盘的STM32控制器通讯，从STM32中获取底盘传感器的状态数据（IMU、超声波、底盘状态数据）；从cmd\_vel 话题上获取小车的速度指令，并将该指令以私有协议的方式下发到底盘STM32中，实现底盘的运动控制；读取电机数据，根据电机反馈来回来的速度或者编码器的转角数据推算小车在世界坐标系下的坐标位置。

完整的ROS节点的代码位于此处 [\[1\]](https://github.com/RuPingCen/mick_robot)。

## 1、底盘ROS节点

### 1.1 ROS节点功能定义

底盘接口需要根据具体需求进行定义，通常小车底盘应当具备以下功能

1、接收其他ROS节点发布在cmd\_vel话题上的速度指令，并转化为控制命令通过串口发送给小车底盘上的STM32控制器，控制小车速度

2、接收小车底盘STM32控制板上传的每个电机的状态信息，根据小车模型推算出小车的位置，并发布小车里程计信息到/odom话题上

3、接收小车底盘STM32控制板上传的IMU、超声波传感器数据以及程序的状态信息。

### 1.2 接收cmd\_vel话题数据

通常来说在ROS框架下的自主导航小车都是用的 **geometry\_msgs/Twist** 这种消息类型来发布小车的速度命令的，而 **geometry\_msgs/Twist** 的数据结构如下，主要包含3个方向的线速度和旋转角速度信息，对于我们所使用的差速轮小车来说，只有X方向的线速度和Z轴方向的角速度，两个自由度。如果是使用麦克纳姆的小车底盘[\[2\]](https://blog.csdn.net/crp997576280/article/details/102026459) 则这里我们可以设置X,Y方向的线速度和Z方向的角速度，此时会有3个角速度。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/4c33ee132c9bda42db81d1ddfb31498e.png#pic_center)  
这部分的功能放在了 **void cmd\_vel\_callback(const geometry\_msgs::Twist::ConstPtr& msg)** 函数中进行实现，该函数从 **geometry\_msgs::Twist** 这个结构体指针中获取X方向的线速度和Z方向的角速度（单位：m/s），然后通过两轮差速模型转化为每个轮子的转速（单位：RPM,转每分），然后调用 **send\_rpm\_to\_chassis(v1,v2,v3,v4)** 函数，按照上篇中与STM32开发板约定的协议下发每个轮子的转速信息。

```cpp
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
//ROS_INFO_STREAM("Write to serial port" << msg->data);
// ostringstream os;
float speed_x,speed_y,speed_w;
float v1=0,v2=0,v3=0,v4=0;

speed_x = msg->linear.x;
speed_y = 0;
speed_w = msg->angular.z;

v1 =speed_x-0.5*WHEEL_L*speed_w;   //左边    //转化为每个轮子的线速度
v2 =v1;
v4 =-(speed_x+0.5*WHEEL_L*speed_w);
v3 =v4;

v1 =v1/(2.0*WHEEL_R*WHEEL_PI);    //转换为轮子的速度　RPM
v2 =v2/(2.0*WHEEL_R*WHEEL_PI);
v3 =v3/(2.0*WHEEL_R*WHEEL_PI);
v4 =v4/(2.0*WHEEL_R*WHEEL_PI);

v1 =v1*WHEEL_RATIO*60;    //转每秒转换到RPM
v2 =v2*WHEEL_RATIO*60;
v3 =v3*WHEEL_RATIO*60;
v4 =v4*WHEEL_RATIO*60;


send_rpm_to_chassis(v1,v2,v3,v4);	 
//send_rpm_to_chassis(200,200,200,200);	
ROS_INFO_STREAM("v1: "<<v1<<"      v2: "<<v2<<"      v3: "<<v3<<"      v4: "<<v4);
ROS_INFO_STREAM("speed_x:"<<msg->linear.x<<"      speed_y:"<<msg->linear.y<<"      speed_w:"<<msg->angular.z);
}
```

### 1.3 发布里程计数据

里程计的发布位于 **void calculate\_position\_for\_odometry(void)** 和 **void publish\_odomtery(float position\_x,float position\_y,float oriention,float vel\_linear\_x,float vel\_linear\_y,float vel\_linear\_w)** 两个函数中， 其中**calculate\_position\_for\_odometry** 函数负责从电机反馈的数据中计算出当前时候小车在odom坐标系下的位置、方向角和速度信息，然后调用 **publish\_odomtery** 函数发布小车的位姿和速度信息到ROS的格式。

注意：这里我们使用的M3508电机带有绝对位置编码器，在使用的过程中我们发现使用绝对位置编码器比使用速度来推算小车位置会更加的准确。

```cpp
float s1=0,s2=0,s3=0,s4=0;
float s1_last=0,s2_last=0,s3_last=0,s4_last=0;
float position_x=0,position_y=0,position_w=0;
void calculate_position_for_odometry(void)
{
  //方法１：　　计算每个轮子转动的位移，然后利用Ｆ矩阵合成Ｘ,Y,W三个方向的位移
  float s1_delta=0,s2_delta=0,s3_delta=0,s4_delta=0;
  float v1=0,v2=0,v3=0,v4=0;
  float position_x_delta,position_y_delta,position_w_delta,position_r_delta;
  float linear_x,linear_y,linear_w;

  if((s1_last == 0 && s2_last == 0&& s3_last==0&&s4_last==0) || (moto_chassis[0].counter ==0))
  {
		s1 =    (moto_chassis[0].round_cnt+(moto_chassis[0].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s2 =    (moto_chassis[1].round_cnt+(moto_chassis[1].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s3 =    (moto_chassis[2].round_cnt+(moto_chassis[2].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s4 =    (moto_chassis[3].round_cnt+(moto_chassis[3].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		
		s1_last=s1;
		s2_last=s2;
		s3_last=s3;
		s4_last=s4;
		
		return ;
  }
  s1_last=s1;
  s2_last=s2;
  s3_last=s3;
  s4_last=s4;
 
  //轮子转动的圈数乘以　N*２*pi*r
  s1 =    (moto_chassis[0].round_cnt+(moto_chassis[0].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s2 =    (moto_chassis[1].round_cnt+(moto_chassis[1].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s3 =    (moto_chassis[2].round_cnt+(moto_chassis[2].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s4 =    (moto_chassis[3].round_cnt+(moto_chassis[3].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
 
  s1_delta=s1-s1_last; //每个轮子位移的增量
  s2_delta=s2-s2_last;
  s3_delta=s3-s3_last;
  s4_delta=s4-s4_last;
  
   // ------------------------------------------------------------------------------------------------------------------------------------------------------------------
   if(abs(s1_delta) < 0.001 )  s1_delta=0;
   if(abs(s2_delta) < 0.001 )  s2_delta=0;
    if(abs(s3_delta) < 0.001 )  s3_delta=0;
   if(abs(s4_delta) < 0.001 )  s4_delta=0;
   
  s1_delta = 0.5*s1_delta+0.5*s2_delta;  
  s4_delta = 0.5*s3_delta+0.5*s4_delta; 

//    if(s1_delta || s2_delta || s3_delta || s4_delta)
//   cout<<"s1_delta:  "<<s1_delta<<"   s2_delta: " <<s2_delta<<"   s3_delta: " <<s3_delta<<"   s4_delta: " <<s4_delta<<endl;
 
  position_w_delta =((s4_delta)- (s1_delta))/float(WHEEL_L); //w 单位为弧度
  if(abs(position_w_delta) < 0.0001)
   position_r_delta=0;
  else
  position_r_delta = ((s4_delta)+(s1_delta))/float(2*position_w_delta);
  position_x_delta=position_r_delta*sin(position_w_delta);
  position_y_delta = position_r_delta*(1-cos(position_w_delta));
   // ------------------------------------------------------------------------------------------------------------------------------------------------------------------
 
  position_x=position_x+cos(position_w)*position_x_delta-sin(position_w)*position_y_delta;
  position_y=position_y+sin(position_w)*position_x_delta+cos(position_w)*position_y_delta;
  position_w=position_w+position_w_delta;
  
  if(position_w>2*WHEEL_PI)
  {
     position_w=position_w-2*WHEEL_PI;	
  }
  else if(position_w<-2*WHEEL_PI)
  {
     position_w=position_w+2*WHEEL_PI;
  }
  else;

  v1 =    (moto_chassis[0].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2;
  v2 =    (moto_chassis[1].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
  v3 =    (moto_chassis[2].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
  v4 =    (moto_chassis[3].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
  
  linear_x = 0.25*v1+ 0.25*v2+ 0.25*v3+ 0.25*v4;
  linear_y = 0;
  linear_w = ((0.5*v3+0.5*v4)-(0.5*v1+0.5*v2))/float(WHEEL_L);
  
  ROS_INFO_STREAM("px:  "<<position_x<<"   py: " <<position_y<<"   pw: " <<position_w*57.3
  <<"  vx:  "<<linear_x<<"   vy: " <<linear_y<<"   vw: " <<linear_w<<endl);
 
  publish_odomtery( position_x,position_y,position_w,linear_x,linear_y,linear_w);
}
```

### 1.4 发布IMU数据

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/0a571945abda228489216c36d9b35efa.png#pic_center)  
小车底盘上安装的MPU9250,通过协议接口将IMU的原始数据上传到ROS节点中，并在函数 **publish\_imu\_mag()** 中我们对底盘上传的IMU和磁力计数据进行了发布

```cpp
void publish_imu_mag(void)
{
	static sensor_msgs::Imu imu_msg;
	static sensor_msgs::MagneticField mag_msg;

	imu_msg.header.stamp = ros::Time::now();
	imu_msg.header.frame_id = "/imu";

	imu_msg.orientation.w = imu_chassis.qw;
	imu_msg.orientation.x = imu_chassis.qx;
	imu_msg.orientation.y = imu_chassis.qy;
	imu_msg.orientation.z = imu_chassis.qz;

	// change to rad/s       0.0010652 = 2000/32768/57.3
	imu_msg.angular_velocity.x = imu_chassis.gx*0.0010652; 
	imu_msg.angular_velocity.y = imu_chassis.gy*0.0010652;
	imu_msg.angular_velocity.z = imu_chassis.gz*0.0010652;

	imu_msg.linear_acceleration.x = imu_chassis.ax/32768.0f*4;
	imu_msg.linear_acceleration.y = imu_chassis.ay/32768.0f*4;
	imu_msg.linear_acceleration.z = imu_chassis.az/32768.0f*4;
	imu_pub.publish(imu_msg);

    broadcaster.sendTransform(tf::StampedTransform(
        tf::Transform(tf::Quaternion(imu_chassis.qx,imu_chassis.qy, imu_chassis.qz,
		 imu_chassis.qw),tf::Vector3(0, 0, 0)),ros::Time::now(),"world", "imu"));

	mag_msg.magnetic_field.x = imu_chassis.mx;
	mag_msg.magnetic_field.y = imu_chassis.my;
	mag_msg.magnetic_field.z = imu_chassis.mz;
	mag_msg.header.stamp = imu_msg.header.stamp;
	mag_msg.header.frame_id = imu_msg.header.frame_id;
	mag_pub.publish(mag_msg);
}
```

### 1.5 发布超声波数据

超声波模块暂时不支持上传，后续添加功能上传超声波的数据  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/5262f6c6a4656a48e24b73aa333ffea8.png#pic_center)

### 1.6 TF坐标系

在底盘中有三个坐标系与小车底盘相关，分别是**base\_link**,**odom**,**laser**。

**base\_link**坐标系主要是表达小车底盘中心

**laser**坐标系主要是想表达激光雷达传感器在base\_link坐标系中的位姿，通常来说激光雷达都是与小车刚性连接的，因此laser与base\_link之间的相对变换是固定的，因而我们可以在launch文件中静态配置传感器与小车底盘的关系

```bash
<launch>
<node pkg="tf" type="static_transform_publisher" name="link1_broadcaster" args="0.0 0 0.7 -3.1415926 0 0 base_link laser 50" />
</launch>
```

上述命令中表达的是laser坐标系到base\_link坐标系（表达在base\_link坐标系）的偏移量X=0 y=0 z=0.7 (单位m), yaw=-π pitch=0 roll=0 单位（弧度）,关于使用四元数传参可以参考 \[3\]，\[4\]

**odom**坐标系是算法根据底盘电机反馈数据而推测出小车的位置。 odom的坐标系是每次计算里程计实现更新的，在函数 **void publish\_odomtery(x,y,vel\_x,vel\_y,vel\_w)** 中

```cpp
void publish_odomtery(float  position_x,float position_y,float oriention,float vel_linear_x,float vel_linear_y,float vel_linear_w)
{
	static tf::TransformBroadcaster odom_broadcaster;  //定义tf对象
	geometry_msgs::TransformStamped odom_trans;  //创建一个tf发布需要使用的TransformStamped类型消息
	geometry_msgs::Quaternion odom_quat;   //四元数变量
	nav_msgs::Odometry odom;  //定义里程计对象
		
	//里程计的偏航角需要转换成四元数才能发布
	odom_quat = tf::createQuaternionMsgFromYaw(oriention);//将偏航角转换成四元数

	//载入坐标（tf）变换时间戳
	odom_trans.header.stamp = ros::Time::now();
	//发布坐标变换的父子坐标系
	odom_trans.header.frame_id = "odom";     
	odom_trans.child_frame_id = "base_link";       
	//tf位置数据：x,y,z,方向
	odom_trans.transform.translation.x = position_x;
	odom_trans.transform.translation.y = position_y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;        
	//发布tf坐标变化
	odom_broadcaster.sendTransform(odom_trans);

	//载入里程计时间戳
	odom.header.stamp = ros::Time::now(); 
	//里程计的父子坐标系
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";       
	//里程计位置数据：x,y,z,方向
	odom.pose.pose.position.x = position_x;     
	odom.pose.pose.position.y = position_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;       
	//载入线速度和角速度
	odom.twist.twist.linear.x = vel_linear_x;
	odom.twist.twist.linear.y = vel_linear_y;
	odom.twist.twist.angular.z = vel_linear_w;    
	//发布里程计
	odom_pub.publish(odom);
}
```

## 参考资料

\[1\] https://github.com/RuPingCen/mick\_robot  
\[2\] https://blog.csdn.net/crp997576280/article/details/102026459  
\[3\] https://blog.csdn.net/tiancailx/article/details/78910317  
\[4\] http://wiki.ros.org/tf#static\_transform\_publisher

上一篇：[开源自主导航小车MickX4（二）ROS底盘运动控制](https://blog.csdn.net/crp997576280/article/details/108475154) 下一篇：[开源自主导航小车MickX4（四）底盘URDF模型](https://blog.csdn.net/crp997576280/article/details/109685109)

**欢迎大家点赞在评论区交流讨论（cenruping@vip.qq.com） O(∩\_∩)O**

或者加群交流（1149897304）  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/e7e3b9b172150d7e0831c2b41f147adc.png#pic_center)