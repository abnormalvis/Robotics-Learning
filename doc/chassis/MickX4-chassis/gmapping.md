#### 开源自主导航小车MickX4（五）gmapping建图

- [1 激光雷达驱动](#1-激光雷达驱动)
- [2 3D点云到2D点云的转换](#2-3d点云到2d点云的转换)
- [3 gmapping 建图](#3-gmapping-建图)
  - [3.1 安装 gmapping库](#31-安装-gmapping库)
  - [3.2 配置 gmapping](#32-配置-gmapping)
  - [3.3 启动 gmapping](#33-启动-gmapping)
  - [3.4 保存地图](#34-保存地图)
- [总结](#总结)
- [参考资料](#参考资料)

本章节主要描述了在室外环境如何把3D激光压缩称为2D通过gmapping建图。

## 1 激光雷达驱动

在读取数据时注意激光雷达的IP地址和你使用的路由器的IP网段以及电脑的IP一致。如果激光雷达是直接连上工控机的，则需要你自己设置工控机的IP为与激光雷达同一网段的静态IP。

这里我们使用的是速腾16线的3D激光雷达，这里主要参考了博客\[1\]对激光雷达的数据进行读取

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/1c33b66706101846765c660fd251e03b.png#pic_center)

这里需要注意一个问题： 速腾16线的3D激光雷达定义的坐标系是基线头指向Y轴的负方向，但是厂家的驱动里面进行了坐标系转换，转换成为了ROS下的右手坐标系。在文档的23页有注释说明。

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/5365cdd1464e87e1e4d3b2c85e017092.png#pic_center)

## 2 3D点云到2D点云的转换

由于gmaping是2D激光SLAM的算法，不能直接使用3D点云建图，因此这里需要使用一个额外的包 **pointcloud\_to\_laserscan**[\[2\]](http://wiki.ros.org/pointcloud_to_laserscan) 将雷达采集到的3D点云压成2D点云。

这里需要注意一个问题，如果启动gmapping的时候出现报错，这主要是由于gmapping的堆栈大小的原因，3D转2D的时候点太密集，gmapping默认的堆栈太小，解决方法有两个：1是下载边缘gmapping的源码，手动把堆栈设置到4096大小。另一个方法就是把 **pointcloud\_to\_laserscan** 包中 angle\_increment 调小一些，这样点就没有那么密，数据量就小了。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/99b7e1cdcc0de3e8c322dd0915b2afdc.png)

我所使用的配置参数如下：

```xml
<?xml version="1.0"?>

<launch>
    <!-- run pointcloud_to_laserscan node -->
    <node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">
        <remap from="cloud_in" to="/rslidar_points"/>
        <rosparam>
            target_frame: rslidar # Leave disabled to output scan in pointcloud frame
            transform_tolerance: 0.01
            min_height: -1.5
            max_height: 20

            angle_min: -3.1415926 # -M_PI
            angle_max: 3.1415926 # M_PI
            angle_increment: 0.03 # M_PI/360.0
            scan_time: 0.1
            range_min: 0.2
            range_max: 100.0
            use_inf: true
            inf_epsilon: 1.0

            # Concurrency level, affects number of pointclouds queued for processing and number of threads used
            # 0 : Detect number of cores
            # 1 : Single threaded
            # 2->inf : Parallelism level
            concurrency_level: 2
        </rosparam>

    </node>
</launch>
```

这里3D转2D的效果并不好，这个包感觉上只是截取了集中一层的线条做映射。[修改以后的包\[4\]](https://github.com/RuPingCen/mick_robot/tree/master/sensor_interface/pointcloud_to_laserscan)  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/c48409ce6ae79cf257b0d15611b96513.png#pic_center)

## 3 gmapping 建图

gmapping使用的条件主要由三个：

+   **激光雷达数据 /scan** （这里我们通过 pointcloud\_to\_laserscan 包把3D雷达压缩成为了2D提供到gmapping中）
+   **里程计数据 /odom 话题**（由底盘发布的里程计数据，这里由底盘节点mick\_bringup节点包提供）
+   **TF变换数据 提供base\_link、odmo和laser 的变换关系**（base\_link和odom之间的变换关系可以由发布/odom的节点提供，而激光雷达和基坐标系也就是 laser和base\_link之间的变换关系可以URDF模型提供，或者是用在launch文件中写入静态的TF变换）

gmapping输出的话题有：/map的地图数据，以及一个获取地图的服务（在建图完成以后保存地图用）

图片来源于[\[3\]](https://blog.csdn.net/zhao_ke_xue/article/details/108944811)  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/715444f433dce59f8aeaf58aa20aaf3b.png)

### 3.1 安装 gmapping库

```bash
sudo apt-get install ros-kinetic-gmapping       #根据ROS的版本更改
```

### 3.2 配置 gmapping

配置好坐标系关系以后，接下来我们需要配置gmapping的运行参数（我自己设置的运行参数如下），新建一个文件夹“mick\_navigation”,在该文件夹中建立名为"launch"的文件夹，编写gmapping.launch文件：

```xml
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

### 3.3 启动 gmapping

接下来我们启动相应的节点实现地图的扫描，首先我们要启动激光雷达、pointcloud\_to\_laserscan和底盘三个节点。这里我们把这几个launch文件写入到一个launch中，让其一起启动。在中新建launch文件 **mickx4\_gmapping.launch** 写入一下内容：

```xml
<?xml version="1.0"?>
<launch>
  <arg name="rviz" default="true"/>
   <!-- chassis -->
  <include file="$(find mick_bringup)/launch/mickx4_bringup.launch" />

   <!-- imu -->
  <include file="$(find imu_driver)/launch/wit_imu.launch" />

   <!-- urdf -->
  <include file="$(find mick_description)/launch/state_publisher.launch" />

   <!-- rslidar -->
  <include file="$(find rslidar_pointcloud)/launch/rs_lidar_16.launch" />
 
   <!-- pointcloud_to_laserscan -->
  <include file="$(find pointcloud_to_laserscan)/launch/rslidar.launch" />

  <!-- Gmapping -->
  <include file="$(find mick_navigation)/launch/gmapping_no_rviz.launch" />
  
 <!-- RViz -->
  <node if="$(arg rviz)" pkg="rviz" type="rviz" name="$(anon rviz)" respawn="false" output="screen" args="-d $(find mick_navigation)/rviz/gmapping.rviz" />


</launch>
```

最后新打开一个终端，启动gmapping节点

```bash
source / catkin_ws/devel/setup.bash
roslaunch mick_navigation mickx4_gmapping.launch
```

这里的场景是一个小范围的场景，小车行走的距离大概是500m左右，建图过程中发现很多问题，比如说在室外使用3D转到2D以后，这个激光的会有很多的噪点，激光数据延时也比较大。在比较空旷的地方转弯会使得车辆转弯估计不准，这是由于我们限定了3D-2D时候激光的测量范围，比如在十字路口的时候就会有一个‘’缺口“”存在，不像在室内环境激光都是一个完整的圈，因此此时旋转车就会使得地图建立的不准。

### 3.4 保存地图

扫描完毕以后你需要运行map\_server节点来保存地图，最后获得两个文件（xxx.png和 xxx.yaml）

```bash
rosrun map_server map_saver -f test
```

.png 主要是地图文件、.yaml 包含有地图的参数，精度之类的。

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/f346e9b0650827d4560d966cd0319587.png)

## 总结

1、3D转2D时候这个包只保留了一条线，如果新建一个节点直接把Z轴压缩，把激光压缩到一个平面上，这样就会有很多条线的包罗在一起，在通过滤波处理一下，不知道效果会不会好一些？  
2、**gamapping在运行的时候依赖odom**，如果odom不准确的话会使得gmapping建图的效果就会很差。 4轮差速模型转弯时候由于轮胎会和地面滑动摩擦，这会造成odom误差变大，尤其是没有加入IMU而直接使用轮子的差速模型计算航向角，以下是一个使用轮子计算yaw角导致odom不准的失败案例。

## 参考资料

\[1\] https://www.jianshu.com/p/4bf7102d661c?utm\_campaign=maleskine&utm\_content=note&utm\_medium=seo\_notes&utm\_source=recommendation  
\[2\] http://wiki.ros.org/pointcloud\_to\_laserscan  
\[3\] https://blog.csdn.net/zhao\_ke\_xue/article/details/108944811

上一篇：[开源自主导航小车MickX4（四）底盘URDF模型](https://blog.csdn.net/crp997576280/article/details/109685109)

下一篇：[开源自主导航小车MickX4（六）cartographer 室外2D建图](https://blog.csdn.net/crp997576280/article/details/109685590)

**欢迎大家点赞在评论区交流讨论（cenruping@vip.qq.com） O(∩\_∩)O**

或者加群交流（1149897304）  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/e7e3b9b172150d7e0831c2b41f147adc.png#pic_center)