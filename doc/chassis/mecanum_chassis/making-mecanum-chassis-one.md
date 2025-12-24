## 4 底盘ROS节点

在上一篇博客中我们实现了利用遥控器控制底盘，接下来我们将编写ROS节点接收”cmd\_vel”的话题，将”cmd\_vel”话题上的数据转化为底盘每个电机的转速并通过串口下发到STM32板上。同时接收下位机传过来的每个电机的转速或者绝对转角（需要有绝对编码器支持），通过麦克纳姆轮逆运动学模型计算出小车在X、Y、W三个方向的位移。

### 4.1 接收cmd\_vel话题命令

首先进入第一部分：cmd\_vel\_callback函数为接收”cmd\_vel”话题的回调函数，由于小车只在平面运动，因此我们只取X,Y方向的速度以及绕Z轴旋转的角速度。首先更加第2章讲解的麦克纳姆轮运动学模型(式1.6和1.7)将X,Y,Z 三个方向的速度转化到每个轮子的线速度（v1,v2,v3,v4）。得到线速度以后，根据轮子的半径将线速度转化为轮子的角速度（单位，弧度/秒）。由于我们使用的是大疆的M3510电机，电机的减速比为1:19，因此轮子的转速和电机的转速之间相差19倍关系，最后我们得到的是电机转速的单位是（弧度每秒）。

由于电机反馈的转速单位是RPM（转/分钟）,为了方便控制，这里我们将单位转化为RPM发送到STM32板上，两者之间的转换关系为：弧度每秒（rad/s）=2\*π\*RPM(转/分钟)/60。 这部分的代码我单独挑选出来放在了下方：

（这部分完整的代码见GITHUB：[https://github.com/RuPingCen/mick\_robot\_bringup](https://github.com/RuPingCen/mick_robot_bringup)）

```
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  float speed_x,speed_y,speed_w;
  float v1=0,v2=0,v3=0,v4=0;
 
  speed_x = msg->linear.x;
  speed_y = msg->linear.y;
  speed_w = msg->angular.z;
  
  v1 =speed_x-speed_y-WHEEL_K*speed_w;       //转化为每个轮子的线速度
  v2 =speed_x+speed_y-WHEEL_K*speed_w;
  v3 =-(speed_x-speed_y+WHEEL_K*speed_w);
  v4 =-(speed_x+speed_y+WHEEL_K*speed_w);

  v1 =v1/(2.0*WHEEL_R*WHEEL_PI);    //转换为轮子的速度(弧度/s)
  v2 =v2/(2.0*WHEEL_R*WHEEL_PI);
  v3 =v3/(2.0*WHEEL_R*WHEEL_PI);
  v4 =v4/(2.0*WHEEL_R*WHEEL_PI);
  
  v1 =v1*WHEEL_RATIO*60;    //转换为电机速度　单位　RPM
  v2 =v2*WHEEL_RATIO*60;
  v3 =v3*WHEEL_RATIO*60;
  v4 =v4*WHEEL_RATIO*60;
    
  send_rpm_to_chassis(v1,v2,v3,v4);	//串口发送函数
}
```

注：**WHEEL\_R** 表示麦克纳姆轮的半径

        **WHEEL\_PI** 表示π常数

        **WHEEL\_RATIO** 表示减速度比

        **WHEEL\_K** 则表示以车为中心轮子X，Y坐标的绝对值之和

通过这个函数我们就可以接受”cmd\_vel”话题上的数据控制底盘运动了。我们利用ROS自带的一个turtlesim包中获取键盘按键的节点来发布控制数据到”cme\_vel”话题上。

我们新开一个终端输入：

```
rosrun turtlesim turtle_teleop_key
```

然后再开一个终端启动底盘节点：

```
rosrun mick_robot mick_robot
```

以下是一个测试效果：（使用turtlesim的时候需要将话题修改为“/turtle1/cmd\_vel”）

![](https://i-blog.csdnimg.cn/blog_migrate/f3f049ffdd2fdc375824f4ae84d30f12.gif)

### 4.2 发布里程计数据

完成ROS底盘还需要发布里程计数据，接下来我们通过读取电机的绝对转角，统计每个电机转过的总角度，并换成每个轮子转动的位移。随后通过麦克纳姆轮逆运动学模型（式1.14）计算在X,Y方向的位移，以及航向角的变化。以下是转化的代码：

```
/*
 * ＠function 利用里程计数据实现位置积分
 * 
 */
float s1=0,s2=0,s3=0,s4=0;
float s1_last=0,s2_last=0,s3_last=0,s4_last=0;
float position_x=0,position_y=0,position_w=0;
void calculate_position_by_odometry(void)
{
  //方法１：　　计算每个轮子转动的位移，然后利用Ｆ矩阵合成Ｘ,Y,W三个方向的位移
  float s1_delta=0,s2_delta=0,s3_delta=0,s4_delta=0;
  float v1=0,v2=0,v3=0,v4=0;
  float K4_1 = 1.0/(4.0*WHEEL_K);
  float position_x_delta,position_y_delta,position_w_delta;
  float linear_x,linear_y,linear_w;

  s1_last=s1;
  s2_last=s2;
  s3_last=s3;
  s4_last=s4;

  //轮子转动的圈数乘以　N*２*pi*r
  s1 =(moto_chassis[0].round_cnt+(moto_chassis[0].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s2 =(moto_chassis[1].round_cnt+(moto_chassis[1].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s3 =(moto_chassis[2].round_cnt+(moto_chassis[2].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s4 =(moto_chassis[3].round_cnt+(moto_chassis[3].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 

  s1_delta=s1-s1_last; //每个轮子转速的增量
  s2_delta=s2-s2_last;
  s3_delta=s3-s3_last;
  s4_delta=s4-s4_last;
  
  //逆运动学模型转化为X、Y方向的位移以及航向角的变化
  position_x_delta= 0.25*s1_delta+ 0.25*s2_delta+ 0.25*s3_delta+ 0.25*s4_delta;
  position_y_delta = -0.25*s1_delta+ 0.25*s2_delta- 0.25*s3_delta+ 0.25*s4_delta;
  position_w_delta = -K4_1*s1_delta-K4_1*s2_delta+K4_1*s3_delta+ K4_1*s4_delta; //w 单位为弧度

  //以上电时候的坐标作为里程计的全局坐标 
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
  linear_y = -0.25*v1+ 0.25*v2- 0.25*v3+ 0.25*v4;
  linear_w = -K4_1*v1-K4_1*v2+K4_1*v3+ K4_1*v4;
  
  cout<<"position_x:  "<<position_x<<"   position_y: " <<position_y<<"   position_w: " <<position_w*57.3<<endl;
  cout<<"linear_x:  "<<linear_x<<"   linear_y: " <<linear_y<<"   linear_w: " <<linear_w<<endl<<endl;
    
  publish_odomtery( position_x,position_y,position_w,linear_x,linear_y,linear_w);
    //方法２;利用轮子的转速来推算
}
```

  函数**publish\_odomtery(....)**的作用是将逆运动学模型计算出来的X，Y坐标以及航向角转换为四元数写入到里程计消息类型中（nav\_msgs::Odometry）

```
void publish_odomtery(float  position_x,float position_y,float oriention,
					  float vel_linear_x,float vel_linear_y,float vel_linear_w)
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

基本上底盘ROS节点的主线就是这样，底盘ROS节点主要有三个必须具备的功能：接收cmd\_vel数据、发布里程计、发布TF变换。这里我们只是做了一个简化的版本，后续再慢慢的升级完善。例如所有的参数都是写死在程序里面，后续还需要通过launch文件传递参数，并添加一些底盘校准机制，同时这里我们直接使用的是轮子的转速来推算航向，这个并不准确，最好是利用陀螺仪+罗盘融合来计算航向角会更加的准确。其次由于2D的激光雷达有盲区，我们需要在底盘上增加超声波的接口并通过ROS节点进行发布。以及机器人的URDF模型等，我们将在后续的版本中进行添加。

ROS节点完整的代码放在了[GITHUB](https://github.com/RuPingCen/mick_robot_bringup)上，欢迎大家下载提问。

## 5 自主导航测试

接下来我们按照之前配置导航包的步骤（[自主导航小车实践（三）](https://blog.csdn.net/crp997576280/article/details/99702835)）配置gmapping和amcl包，然后进行地图扫描，这里需要注意一点：我们在使用gmapping的时候是需要分布/scan 、/odom 、 /tf(base\_link和odom之间的变换)、以及/tf(base\_link和laser之间的变换) ,前面三个都是底盘会发布的，最后一个一般是根据URDF加载时候发布的，这里我们还没有写机器人3D模型文件，因此我们需要更加安装位置手动发布激光雷达(laser)和底盘（base\_link）之间的变换信息（所谓变换就是两个坐标系之间的额相对旋转和平移）。

```
 <node pkg="tf" type="static_transform_publisher" name="link1_broadcaster" args="0.0 0 0.5 -3.1415926 0 0 base_link laser 50" />
```

这个参数的传入顺序是（x y z qx qy qz qw frame\_id child\_frame\_id  period\_in\_ms）最后一个参数是发布的频率。

在配置move\_base包的时候需要根据底盘的尺寸修改**local\_costmap\_params.yaml**和**global\_costmap\_params.yaml**两个中的**footprint**参数，使之适应新底盘，其次是两个文件中的**inflation\_radius**来指明小车距离边界的安全距离，然后其他参数与之前保持默认即可。对于**dwa\_local\_planner\_params.yaml**文件中XY方向的加速度和速度参数（**acc\_lim\_x**、**acc\_lim\_y**,**max\_vel\_x**,**max\_vel\_y**）进行限制，传统的轮式小车是将Y方向的加速度限制为0的，这里我们需要进行修改。需要修改的参数基本就这些，其他的与之前的博客保持一样就可以了，至少跑起来是没有问题的。

![](https://i-blog.csdnimg.cn/blog_migrate/1493ef3119a7e4bb1385a0e0c6a8f208.gif)

高清视频请移步**[mick\_自主导航-高清](https://www.bilibili.com/video/av71055707)**观看：[https://www.bilibili.com/video/av71055707](https://www.bilibili.com/video/av71055707)

欢迎邮件或留言讨论cenruping@vip.qq.com