这一部分中我们首先介绍自主导航小车的软件框架，随后介绍每一个部分的配置过程。（本章节主要分析自主导航小车的算法框架，文字表述较大，希望大家耐心读完！）

### 2.1 整体分析

实现自主导航小车一般分为三个部分：建图、路径规划、控制。建图指通过激光雷达或者视觉相机的数据建立2D栅格地图或者是3D点云地图，其中基于2D激光雷达建图的算法一般有Gmapping、Hector、Cartograph。3D激光雷达的建图算法一般是采用谷歌的Cartographer算法。这些算法都已经集成到了ROS中作为一个功能包。路径规划则是在已有的地图中计算得到一条代价最小的路径（这个路径也可以由你来画），控制则是控制小车按照预设的轨迹行驶实现轨迹跟踪。

Ros中已经有了自己的自主导航包集合（这张万年不变的老图），ROS官方给出的这张图非常的经典，基本包含了自主导航小车的框架，我们按照自主导航小车的功能块来分析这个图。![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/784ed023735934ebfb311bfc18e52c58.png#pic_center)

1、假设我们已经通过gmapping或者其他的算法获得了一张点云图,以2D地图为例（通常会得到xxx.pgm和xxx.yaml两个文件）。接下来我们利用扫描得到的地图进行自主导航，

2、我们利用map\_server这个包加载已有的导航地图并将其转换为导航可用的图(具体来说就是costmap)，这个costmap提供地图中每个点的代价。也就是说costmap是map\_server这个节点提供地图的类型。map\_server这个节点对外提供了2张2维3层的地图，其中2张是指global\_costmap和local\_costmap, 3层包括staticlayer(静态层)、obstaclelayer(动态层)、inflationlayer(膨胀层)。

3、第二个要素是定位，在基于激光导航的小车中定位数据来源有两个地方轮式里程计和AMCL包。其中AMCL(称为自适应蒙特卡洛定位)，这个包主要是利用当前激光雷达获取到的数据在已有的地图中进行搜索匹配，定位出小车在地图中的位置。AMCL包输出的是一个在"odom"和"map"坐标之间的TF变换（旋转和平移量）。注意：在自主导航中传感器之间的位置关系以及小车在地图中的位置均是通过TF变换来表达的，比如说轮式里程计输出的"odom"和"base\_link"之间的变换，底盘与激光雷达的相对安装位置则是通过"base\_link"和"laser"之间的TF来表示的。这样就可以形成一颗TF树，通过TF树可以找到树上任意两个节点之间的TF变换（即旋转和平移闭环）。

4、导航所需要的地图和机器人当前的位置都知道以后，我们可以开始规划路径了，move\_base这个包由全局规划器和局部规划器组成，全局规划器接收由map\_server提供的global\_costmap地图（地图类型依旧是costmap）规划出全局路径，这里用到的算法通常是A*和D*算法。局部规划器在local\_costmap地图上动态规划，实现避障、绕行等功能，其中所涉及到的算法为人工势场法和DWA算法.  
move\_base这个包含了局部规划器和全局规划器，这个包需要输入机器人当前的速度信息（里程计输出的"odom"和"base\_link"之间的变换）和机器人在地图中的位置（由ACML模块输出的"odom"和"map"之间的TF变换）。计算输出底盘速度和转角控制参数到"/cmd\_vel"这个topic上，最后由机器人底盘节点从这个topic上接收速度和角度控制指令，对底盘实现闭环控制。

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/86784c76ca7798ca2d4dad9badfe691b.png#pic_center)  
小结，总体来讲自主导航小车的算法结构如上图所示。建图、定位、路径规划、控制是实现自主导航小车的四个基本要素。其中运动规划主要由move\_base包提供，到这里我们基本上搞清楚了自主导航小车的算法框架及其组成部分，接下来我们按步骤配置每一个模块，将小车实现建图和路径规划。

### 2.2 驱动包配置

首先我们配置小车的工控机上传感器的ROS驱动包。我们实验所使用的小车上一共使用了思岚2D的激光雷达、北通遥控手柄两个额外的传感器，其次先锋机器人的底盘也需要一个ROS驱动包，因此我们需要配置三个驱动包。首先我们在工作空间下创建一个“pioneer3at\_pkg”文件夹专门存所有导航所需要的ROS包以及配置文件。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/a8f0a38ddbf9e244b37f0ed21587ede8.png#pic_center)  
1、激光雷达和游戏手柄的包比较简单，只需要从Github上clone 下来，直接编译就行。  
**激光雷达：**

```
cd ~/catkin_ws/src/ pioneer3at_pkg

git clone https://github.com/ncnynl/rplidar_ros.git   #激光雷达驱动包
cd catkin_ws/src/pioneer3at_pkg/rplidar_ros/scripts  #拷贝规则文件
sudo cp rplidar.rules /etc/udev/rules.d/

cd ~/catkin_ws/
catkin_make
source / catkin_ws/devel/setup.bash
```

**游戏手柄：**

```
sudo apt-get install ros-kinetic-joystick-*
```

2、先锋机器人的底盘驱动包请参考[我之前的博客](https://blog.csdn.net/crp997576280/article/details/98470571)第2.1-2.8部分。  
3、由于手柄的消息类型与底盘的控制节点接受的topic（/cms\_vel）的类型不一致，无法直接转发到底盘的topic上，因此我们编写如下节点（命名为：“transfor\_joy”），实现转发手柄的数据到机器人底盘上，实现通过手柄控制先锋机器人的底盘。[代码见此处](https://download.csdn.net/download/crp997576280/11576238)  
**源文件如下：**

```
#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h> 

using namespace std;

ros::Publisher vel_pub_;
ros::Subscriber joy_sub_; 
  
double l_scale_=0.5, a_scale_=0.5; 


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) 
{ 
	geometry_msgs::Twist twist;  
	if(joy->buttons[2])
	{
		if(joy->buttons[0])
		{
			l_scale_=0.5;
			a_scale_=0.5;
		}
		else if(joy->buttons[1])
		{
			l_scale_=1;
			a_scale_=1;
		}
		else;
		twist.angular.z = a_scale_*joy->axes[0]; 
		twist.linear.x = l_scale_*joy->axes[1]; 
	}
	else
	{
		twist.angular.z = 0; 
		twist.linear.x = 0; 
	} 

	vel_pub_.publish(twist);
	//cout<<"I recived the data: "<<twist.linear.x <<"  "<<twist.angular.z <<endl;

}

int main(int argc, char** argv) 
{ 
 	ros::init(argc, argv, "transfor_joy");
 	ros::NodeHandle nh_; 
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback); 

	ros::spin();
}
```

同时创建launch文件"**transfor\_joy.launch** "

```
<launch>
  <node pkg="transfor_joy" type="transfor_joy" name="transfor_joy">

  <!-- remap from="/cmd_vel" to="/turtle1/cmd_vel"/-->
  </node>

  <!-- Joy node -->
  <node pkg="joy" type="joy_node" name="joy_node">

    <param name="dev" value="/dev/input/js0" type="string "/>
    <param name="deadzone" value="0.05" type="double"/>    
    <param name="autorepeat_rate" value="10" type="double"/> 

  </node>
 </launch>
```

4、编译通过以后，我们修改p3at\_bringup包中的launch文件，同时启动底盘驱动、手柄、激光雷达以及手柄数据转发的节点。（当然你也可以自己编写一个新的launch文件）

```
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!-- Load URDF file -->
  <include file="$(find p3at_description)/launch/state_publisher.launch" />

  <!-- Load robot driver -->
  <node name="RosAria" pkg="rosaria" type="RosAria">
    <param name="port" value="/dev/pioneer3at" />
    <remap from="/RosAria/pose" to="/odom"/>
    <remap from="/RosAria/cmd_vel" to="/cmd_vel"/>
  </node>

   <include file="$(find rplidar_ros)/launch/rplidar_a3.launch" />
  
  <include file="$(find transfor_joy)/launch/transfor_joy.launch" />
  
</launch>
```

最后在工作空间下执行：

```
source / catkin_ws/devel/setup.bash
roslaunch p3at_bringup p3at_bringup.launch
```

到这里你可以通过遥控器控制机器人底盘了，你可以自己启动一个rviz订阅激光雷达的数据topic，检测激光雷达输出是否正常。如果你使用了其他的机器人底盘或者是自制的机器人底盘，只需要将我们配置的底盘包取代为你自己的底盘驱动包就可以了。

在小节中我们已经实现了小车上所有传感器数据的读取，这里我们将开始配置建图包和导航包。这一个小节中我们要实现利用gmapping算法扫描空间地图。

### 2.3 建图包配置

Gmapping运行时候涉及到四个坐标系的关系,他们分别是"odom"、“base\_link”、“laser”、“map”。“odom"和"base\_link"之间的TF变换一般是由底盘输出提供，表示里程计的运动信息，这个TF信息在我们启动底盘节点（p3at\_bringup）的时候由这个节点发布。  
而"base\_link"和"laser"之间的关系是指底盘中心和激光雷达的位置关系，这个是根据我们的安装位置来决定的，需要我们自己去发布一个这样的TF关系，或者在机器人的描述文件中设定这个TF。这样我们通过TF树就可以得到"odom”、“base\_link”、"laser"三个中任意两个的变换关系了，而"odom"和"map"这个关系就是由gmapping来输出的了。  
这里我们提到了一个机器人描述文件这个东西，一般来说我们买来的标准实验平台都会提供一个名为"xxx\_description"的ROS包，这个包提供了URDF下的机器人3D模型。（你可以在网上找到和你购买的机器人相对应的模型包，如果你是自己设计的机器人，这个需要你自己编写）  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/6d74c17ab62b4a2808852c65b50ae0a3.png#pic_center)  
我们可以通过ROS节点启动的命令启动这个包就可以看到这个机器人的3D模型  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/666ff5be6d53970c85b092eb868f3667.png#pic_center)  
然后修改其中激光雷达和底盘中心的变换关系（即设置"base\_link"和"laser"之间的TF关系）。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/f7a9d39d3dc0940f32653e74678ec946.png#pic_center)  
配置好坐标系关系以后，接下来我们需要配置gmapping的运行参数（我自己设置的运行参数如下），新建一个文件夹“p3at\_navigation”,在该文件夹中建立名为"launch"的文件夹，编写gmapping.launch文件：

```
<?xml version="1.0"?>
<launch>

  <arg name="scan_topic" default="scan" />

  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">

    <param name="odom_frame" value="odom"/>
    <param name="base_frame" value="base_link"/>
    <param name="map_frame" value="map"/>

    <!-- Process 1 out of every this many scans (set it to a higher number to skip more scans)  -->
    <param name="throttle_scans" value="1"/>

    <param name="map_update_interval" value="5.0"/> <!-- default: 5.0 -->

    <!-- The maximum usable range of the laser. A beam is cropped to this value.  -->
    <param name="maxUrange" value="5.0"/>

    <!-- The maximum range of the sensor. If regions with no obstacles within the range of the sensor should appear as free space in the map, set maxUrange < maximum range of the real sensor <= maxRange -->
    <param name="maxRange" value="10.0"/>

    <param name="sigma" value="0.05"/>
    <param name="kernelSize" value="1"/>
    <param name="lstep" value="0.05"/>
    <param name="astep" value="0.05"/>
    <param name="iterations" value="5"/>
    <param name="lsigma" value="0.075"/>
    <param name="ogain" value="3.0"/>
    <param name="minimumScore" value="0.0"/>
    <!-- Number of beams to skip in each scan. -->
    <param name="lskip" value="0"/>

    <param name="srr" value="0.01"/>
    <param name="srt" value="0.02"/>
    <param name="str" value="0.01"/>
    <param name="stt" value="0.02"/>

    <!-- Process a scan each time the robot translates this far  -->
    <param name="linearUpdate" value="0.1"/>

    <!-- Process a scan each time the robot rotates this far  -->
    <param name="angularUpdate" value="0.05"/>

    <param name="temporalUpdate" value="-1.0"/>
    <param name="resampleThreshold" value="0.5"/>

    <!-- Number of particles in the filter. default 30        -->
    <param name="particles" value="10"/>

<!-- Initial map size  -->
    <param name="xmin" value="-10.0"/>
    <param name="ymin" value="-10.0"/>
    <param name="xmax" value="10.0"/>
    <param name="ymax" value="10.0"/>

    <!-- Processing parameters (resolution of the map)  -->
    <param name="delta" value="0.02"/>

    <param name="llsamplerange" value="0.01"/>
    <param name="llsamplestep" value="0.01"/>
    <param name="lasamplerange" value="0.005"/>
    <param name="lasamplestep" value="0.005"/>

    <remap from="scan" to="$(arg scan_topic)"/>
  </node>
</launch>
 
```

接下来我们启动相应的节点实现地图的扫描：  
首先我们要启动激光雷达、摇杆和底盘三个节点，由于我们之前在2.4中修改过p3at\_bringup的launch文件，使得其可以同时启动这三个传感器，因此这里我们直接使用p3at\_bringup命令启动三个节点

```
source / catkin_ws/devel/setup.bash
roslaunch p3at_bringup p3at_bringup.launch
```

新打开一个终端，随后启动机器人的3D模型，提供"base\_link"和"laser"之间的TF关系并且让RVIZ中可以看到机器人模型。

```
source / catkin_ws/devel/setup.bash
roslaunch p3at_description state_publisher.launch
```

最后新打开一个终端，启动gmapping节点

```
source / catkin_ws/devel/setup.bash
roslaunch p3at_ navigation gmapping.launch
```

这时候你就可以通过摇杆遥控机器人运行了，通过激光雷达获取数据扫描空间中的地图了。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/e12d2c8b6efe017eb8c294746ed54256.gif)  
扫描完毕以后你需要运行map\_server节点来保存地图，最后获得两个文件（xxx.png和 xxx.yaml）

```
rosrun map_server map_saver -f mymap 
```

.png 主要是地图文件、.yaml 包含有地图的参数，精度之类的。

注：这里我们没有设置move\_base因此只能通过遥控器来遥控小车行走进行扫图，当你看完后续的教程以后可以将move\_base包和gmapping包一起运行，这样就可以通过rviz设定一个目标点让机器人自己走过去，一边规划一边建图。

欢迎留言讨论或 cenruping@vip.qq.com  
代码见GitHub：https://github.com/RuPingCen/P3AT\_Auto\_Navigation

上一篇： [自主导航小车实践（一）](https://blog.csdn.net/crp997576280/article/details/98240613) 下一篇： [自主导航小车实践（三）](https://blog.csdn.net/crp997576280/article/details/99702835)