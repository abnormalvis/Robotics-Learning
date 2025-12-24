#### 开源自主导航小车MickX4（九）move\_base 导航框架

- [1 move\_base 导航框架整体分析](#1-move_base-导航框架整体分析)
  - [1.1 map\_server](#11-map_server)
  - [1.2 AMCL定位](#12-amcl定位)
  - [1.3 move\_base 包](#13-move_base-包)
- [2 AMCL 定位 配置](#2-amcl-定位-配置)
  - [2.1 map\_server 发布地图](#21-map_server-发布地图)
  - [2.2 AMCL 定位](#22-amcl-定位)
- [3 move\_base配置](#3-move_base配置)
- [4 测试视频](#4-测试视频)
- [参考资料](#参考资料)

目前开源的导航框架有 百度的 Applo，autoware和move\_base。move\_base的框架资料较多，上手比较容易，适用于激光传感器在小车上实现自主导航，move\_base相比其他两个框架，其本身没有融合有感知模块，需要自己添加模块和定义接口。这一部分中我们首先介绍自主导航小车的软件框架，随后介绍每一个部分的配置过程。（本章节主要分析自主导航小车的算法框架，文字表述较大，希望大家耐心读完！）

## 1 move\_base 导航框架整体分析

实现自主导航小车一般分为三个部分：建图、路径规划、控制。建图指通过激光雷达或者视觉相机的数据建立2D栅格地图或者是3D点云地图，其中基于2D激光雷达建图的算法一般有Gmapping、Hector、Cartograph。3D激光雷达的建图算法有谷歌的Cartographer算法和LeGO-LOAM。这些算法很多已经集成到了ROS中作为一个功能包。路径规划则是在已有的地图中计算得到一条代价最小的路径（这个路径也可以由你来画），控制则是控制小车按照预设的轨迹行驶实现轨迹跟踪。

ROS中已经有了自己的自主导航包集合（这张万年不变的老图），ROS官方给出的这张图非常的经典，基本包含了自主导航小车的框架，我们按照自主导航小车的功能块来分析这个图。![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/784ed023735934ebfb311bfc18e52c58.png#pic_center)

### 1.1 map\_server

假设我们已经通过gmapping或者其他的算法获得了一张点云图,以2D地图为例（通常会得到xxx.pgm和xxx.yaml两个文件）。接下来我们利用扫描得到的地图进行自主导航。我们利用map\_server这个包加载已有的导航地图并将其转换为导航可用的图(具体来说就是costmap)，这个costmap提供地图中每个点的代价。

如下图所示，map\_server主要是加载我们之前扫描到的地图发布出costmap地图,map\_server这个节点对外提供了2张2维3层的地图，其中2张是指global\_costmap和local\_costmap。一张用于move\_base中的全局规划器（global\_planner）,另外一张用于move\_base中的局部规划器（local\_planner）。每张地图有三层（staticlayer(静态层)、obstaclelayer(动态层)、inflationlayer(膨胀层)）。map\_server发布"/map"和"/map\_metadata"两个topic, "/map"则被AMCL接收用于匹配激光传感器数据 。 /map\_metadata 被move\_base接收。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/98a8d07c3818afc9a033833417f55f65.png#pic_center)

### 1.2 AMCL定位

在基于激光导航的小车中定位数据来源有两个地方轮式里程计和AMCL包。其中AMCL(称为自适应蒙特卡洛定位)，这个包主要是利用当前激光雷达获取到的数据在已有的地图中进行搜索匹配，定位出小车在地图中的位置。AMCL接收当前激光雷达的数据（/scan）、TF变换树和初始位姿估计（/Initialpose），在map\_server发布的地图中 /map 匹配定位机器人在地图中的位置，AMCL包估计地图与里程计（即"odom"和"map “）之间的TF变换，形成一个误差项，修正"odom"坐标系。AMCL会将定位数据发布在TF树和”/amcl\_pose"这个topic中。

注意：在自主导航中传感器之间的位置关系以及小车在地图中的位置均是通过TF变换来表达的，比如说轮式里程计输出的"odom"和"base\_link"之间的变换，底盘与激光雷达的相对安装位置则是通过"base\_link"和"laser"之间的TF来表示的。这样就可以形成一颗TF树，通过TF树可以找到树上任意两个节点之间的TF变换（即旋转和平移闭环）。

### 1.3 move\_base 包

导航所需要的地图（**map\_server**）和机器人当前的位置(**amcl**)都知道以后，我们可以开始规划路径了，move\_base这个包由全局规划器和局部规划器组成，全局规划器接收由map\_server提供的global\_costmap地图（地图类型依旧是costmap）规划出全局路径，这里用到的算法通常是A*和D*算法。局部规划器在local\_costmap地图上动态规划，实现避障、绕行等功能，其中所涉及到的算法为人工势场法和DWA算法。move\_base这个包含了局部规划器和全局规划器，这个包需要输入机器人当前的速度信息（里程计输出的"odom"和"base\_link"之间的变换）和机器人在地图中的位置（由ACML模块输出的"odom"和"map"之间的TF变换）。计算输出底盘速度和转角控制参数到"/cmd\_vel"这个topic上，最后由机器人底盘节点从这个topic上接收速度和角度控制指令，对底盘实现闭环控制。

小结，总体来讲自主导航小车的算法结构如上图所示。建图、定位、路径规划、控制是实现自主导航小车的四个基本要素。其中运动规划主要由move\_base包提供，到这里我们基本上搞清楚了自主导航小车的算法框架及其组成部分，接下来我们按步骤配置每一个模块，将小车实现建图和路径规划。

## 2 AMCL 定位 配置

### 2.1 map\_server 发布地图

新打开一个终端，启动map\_server ,测试地图我们放在了[代码目录](https://github.com/RuPingCen/mick_robot)下的 mick\_robot/mick\_navigation/map中

```bash
source / catkin_ws/devel/setup.bash
rosrun map_server map_server /home/administrator/catkin_ws/src/mick_robot/mick_navigation/map/cqu_lab.yaml
```

如果提示没有 map-server的话可以通过以下命令安装

```bash
sudo apt-get install ros-kinetic-map-server
```

map-server 启动以后会有一个 **/map** 和 **/map\_metadata** 两个topic发出。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/1974688faddabd914ff83010b5930658.png#pic_center)

### 2.2 AMCL 定位

这里我们使用AMCL这个模块进行实验，先把导航框架的数据通道打通。在后面可以切换成cartographer的定位算法或者其他的定位算法。这里我们在文件夹“mick\_navigation/launch”中建立名为" amcl.launch"的文件,对AMCL模块进行配置：

```
<launch>
  <arg name="use_map_topic" default="false"/>
  <arg name="scan_topic" default="scan" />

  <node pkg="amcl" type="amcl" name="amcl">
    <param name="use_map_topic" value="$(arg use_map_topic)"/>
    <!-- Publish scans from best pose at a max of 10 Hz -->
    <param name="odom_model_type" value="omni-corrected"/>
    <param name="odom_alpha5" value="0.1"/>
    <param name="gui_publish_rate" value="10.0"/>
    <param name="laser_max_beams" value="720"/>
    <param name="laser_min_range" value="0.1"/>
    <param name="laser_max_range" value="30.0"/>
    <param name="min_particles" value="500"/>
    <param name="max_particles" value="2000"/>
    <!-- Maximum error between the true distribution and the estimated distribution. -->
    <param name="kld_err" value="0.05"/>
    <param name="kld_z" value="0.99"/>
    <param name="odom_alpha1" value="0.2"/>
    <param name="odom_alpha2" value="0.2"/>
    <!-- translation std dev, m -->
    <param name="odom_alpha3" value="0.2"/>
    <param name="odom_alpha4" value="0.2"/>
    <param name="laser_z_hit" value="0.5"/>
    <param name="laser_z_short" value="0.05"/>
    <param name="laser_z_max" value="0.05"/>
    <param name="laser_z_rand" value="0.5"/>
    <param name="laser_sigma_hit" value="0.2"/>
    <param name="laser_lambda_short" value="0.1"/>
    <param name="laser_model_type" value="likelihood_field"/>
    <!-- Maximum distance to do obstacle inflation on map, for use in likelihood_field model. -->
    <param name="laser_likelihood_max_dist" value="2.0"/>
    <!-- Translational movement required before performing a filter update.  -->
    <param name="update_min_d" value="0.1"/>
    <!--Rotational movement required before performing a filter update. -->
    <param name="update_min_a" value="0.314"/>
    <param name="odom_frame_id" value="odom"/>
    <param name="base_frame_id" value="base_link"/>
    <param name="global_frame_id" value="map"/>
    <!-- Number of filter updates required before resampling. -->
    <param name="resample_interval" value="1"/>
    <!-- Increase tolerance because the computer can get quite busy -->
    <param name="transform_tolerance" value="1.0"/>
    <!-- Exponential decay rate for the slow average weight filter, used in deciding when to recover by adding random poses. A good value might be 0.001. -->
    <param name="recovery_alpha_slow" value="0.0"/>
    <!--Exponential decay rate for the fast average weight filter, used in deciding when to recover by adding random poses. A good value might be 0.1. -->
    <param name="recovery_alpha_fast" value="0.1"/>
    <!-- Initial pose mean -->
    <param name="initial_pose_x" value="0.0" />
    <param name="initial_pose_y" value="0.0" />
    <param name="initial_pose_a" value="0.0" />
    <!-- When set to true, AMCL will subscribe to the map topic rather than making a service call to receive its map.-->
    <param name="receive_map_topic" value="true"/>
    <!--  When set to true, AMCL will only use the first map it subscribes to, rather than updating each time a new one is received. -->
    <param name="first_map_only" value="false"/>
    <remap from="scan" to="$(arg scan_topic)"/>
  </node>

</launch>
```

再新打开一个终端，启动AMCL节点

```
source / catkin_ws/devel/setup.bash
roslaunch mick_navigation amcl.launch
```

这时候我们新开一个终端启动RVIZ添加“/map”的topic 就可以看到我们在（[开源自主导航小车MickX4（六）cartographer 室外2D建图](https://blog.csdn.net/crp997576280/article/details/109685590)）中建好的2D 地图了  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/483062e409ab802891aed8575b89274a.png#pic_center)

注意：amcl节点如果提示没有map和base\_link的有效数据，这是由于底盘没有发布有odom坐标系的TF。只需要启动底盘节点 激光雷达和URDF模型就可以了。

```bash
source / catkin_ws/devel/setup.bash
roslaunch mick_bringup mickx4_bringup_v2.launch
```

这时候我们再启动底盘的节点，并RVIZ中点击一下“2D Pose Estimate”就可以正常定位了

## 3 move\_base配置

接下来开始配置move\_base模块，move\_base模块需要有五个文件分别是：

+   local\_costmap\_params.yaml（局部costmap参数文件）
+   global\_costmap\_params.yaml（全局地图参数文件）
+   dwa\_local\_planner\_params.yaml(局部规划器参数文件)
+   global\_planner\_params.yaml（全局规划器参数文件）
+   move\_base\_params.yaml（move\_base运行参数）。

其中xxx\_costmap\_params.yaml两个文件主要是配置地图的，比如地图的膨胀区域。xxx\_ planner\_params.yaml两个文件主要的规划器的运行参数。基本上这些参数按照wiki上的教程进行配置能运行起来，但是运行的不太好，这里面的参数很多还需要进一步理解。最好的方法是找一个开源的框架，模仿他的配置方法，然后在这个基础上对参数进行修改，然后对照文档理解参数的含义。

我在配置的时候主要参考了 [navguide.pdf 文档 \[5\]](http://zkytony.com/documents/navguide.pdf), 文档是英文的，也有[博客\[4\]](https://blog.csdn.net/tobebest_lah/article/details/93633163)进行了翻译梳理。此外，这两篇博客：[\[2\]](https://blog.csdn.net/qq_25241325/article/details/81207278)[\[3\]](https://blog.csdn.net/forrest_z/article/details/62891055)也可以进行参考理解其中参数的意思。

在文件夹“mick\_navigation/launch”中建立名为" mickx4\_amcl.launch"的文件，将所有的节点放在一个launch文件中启动：

```xml
<?xml version="1.0"?>
<launch>
  <arg name="rviz" default="true"/>

   <!-- chassis -->
  <include file="$(find mick_bringup)/launch/mickx4_bringup_v2.launch" />
  
   <!-- urdf -->
  <include file="$(find mick_description)/launch/state_publisher.launch" />
  
   <!-- imu -->
  <!-- include file="$(find imu_driver)/launch/wit_imu.launch" /-->
  
   <!-- xsens  -->
  <include file="$(find xsens_driver)/launch/xsens_driver.launch" />
  
   <!-- rslidar -->
  <include file="$(find rslidar_pointcloud)/launch/rs_lidar_16.launch" />
 
   <!-- pointcloud_to_laserscan -->
  <include file="$(find pointcloud_to_laserscan)/launch/rslidar.launch" />

  <!-- Map Server -->
 <arg name="map_file" default="$(find mick_navigation)/map/cartograph_test.yaml"/>
 <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

  <!-- AMCL -->
 <include file="$(find mick_navigation)/launch/include/amcl.launch.xml" />


 <!-- Move Base -->
 <include file="$(find mick_navigation)/launch/include/move_base_mickx4.launch.xml" />

  <!-- RViz -->
  <node if="$(arg rviz)" pkg="rviz" type="rviz" name="$(anon rviz)" respawn="false" output="screen" args="-d $(find mick_navigation)/rviz/localization_mickx4.rviz" />

</launch>
```

配置好move\_base以后，新开启一个终端启动mickX4差速底盘、传感器、move\_base 、map\_server以及amcl 导航模组 。([我所使用的参数文件\[6\]](https://github.com/RuPingCen/mick_robot/blob/master/mick_navigation/launch/mickx4_amcl.launch))

```bash
source / catkin_ws/devel/setup.bash
roslaunch mick_navigation mickx4_amcl.launch
```

注意：AMCL需要手动设置初始化点（通过点击RVIZ上面的箭头 **2D Pose Estimate**），初始化成功以后，可以看到激光扫描的线是和地图的边缘对齐的，小车周围也会有一对的带箭头的线（粒子）。这时如果move\_base配置正确，再点击 箭头（**2D Nav Goal**）再地图上就会出现一个红色的线（全局规划路径），靠近车前面会有一段绿色的路径（局部规划路径）。这时候把遥控器上的模式切换开关切换到自主模式下，底盘就可以运动了。

注意小车必须要要有局部规划路径才可以动，也就是绿色的曲线。

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/1bafbdc0338c529bff5b45f3b1f7f814.png#pic_center)

## 4 测试视频

配置导航的内容基本上就是这些，如果还需要设置开机自启动和上位机控制，可以参考我的其他博客文章配置。

[ROS节点开机自启动](https://blog.csdn.net/crp997576280/article/details/108272412)  
[ROS上位机界面-nav2djs导航组件](https://blog.csdn.net/crp997576280/article/details/102781371)

## 参考资料

\[1\] https://github.com/RuPingCen/mick\_robot  
\[2\] https://blog.csdn.net/qq\_25241325/article/details/81207278  
\[3\] https://blog.csdn.net/forrest\_z/article/details/62891055  
\[4\] http://zkytony.com/documents/navguide.pdf  
\[5\] https://blog.csdn.net/tobebest\_lah/article/details/93633163  
\[6\] https://github.com/RuPingCen/mick\_robot/blob/master/mick\_navigation/launch/mickx4\_amcl.launch

上一篇：[开源自主导航小车MickX4（八）LeGo-LOAM 室外3D建图](https://blog.csdn.net/crp997576280/article/details/111657554)

下一篇：[开源自主导航小车MickX4（十）总结](https://blog.csdn.net/crp997576280/article/details/113438074)

*欢迎大家点赞在评论区交流讨论（cenruping@vip.qq.com） O(∩\_∩)O*\*

或者加群交流（1149897304）  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/32fa6571126eb1eba4d079c3496b87d6.png)