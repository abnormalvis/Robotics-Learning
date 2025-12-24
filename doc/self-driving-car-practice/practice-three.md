在上一篇博客中我们实现了遥控手柄控制先锋机器人移动，同时使用机器人自身携带的激光雷达进行地图扫描。在这一篇博客中我们将利用扫描到的地图进行导航以及路径规划。这里主要涉及map\_server包、AMCL包、move\_base包集合三个模块。

### 2.5 导航包配置

如下图所示，map\_server主要是加载我们之前扫描到的地图发布出[costmap地图](https://blog.csdn.net/sunyoop/article/details/78183145),在自主导航小车中map\_server对外提供两张costmap，一张用于move\_base中的全局规划器（global\_planner）,另外一张用于move\_base中的局部规划器（local\_planner）。每张地图有三层（静态层、动态层、膨胀层）。map\_server发布"/map"和"/map\_metadata"两个topic, "/map"则被AMCL接收用于匹配激光传感器数据。

AMCL全称为“自适应蒙特卡洛定位”，AMCL接收当前激光雷达的数据（/scan）、TF变换树和初始位姿估计（/Initialpose），在map\_server发布的地图中匹配定位机器人在地图中的位置，AMCL包估计地图与里程计（即"odom"和"map “）之间的TF变换，形成一个误差项，修正"odom"坐标系。AMCL会将定位数据发布在TF树和”/amcl\_pose"这个topic中。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/a40fbaebb02d55d264f510fe852d2c5a.png)  
move\_base则接收里程计和AMCL提供的TF树，通过内部的全局规划器和局部规划器计算的到机器人移动的线速度和角速度，通过"/cmd\_vel"这个topic发布给底盘节点，控制机器人。

这里我们在文件夹“p3at\_navigation/launch”中建立名为" amcl.launch"的文件,对AMCL模块进行配置：

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

新打开一个终端，启动map\_server

```
source / catkin_ws/devel/setup.bash
rosrun map_server map_server ~/catkin_ws/src/ pioneer3at_pkg /p3at_navigation /map/cqu_lab.yaml
```

再新打开一个终端，启动AMCL节点

```
source / catkin_ws/devel/setup.bash
roslaunch p3at_navigation amcl.launch
```

这时候我们新开一个终端启动RVIZ添加“/map”的topic 就可以看到地图了，由于底盘没有发布有odom坐标系的TF，因此amcl节点会提示没有map和base\_link的有效数据。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/897912298b7c27f2aa911b649b3da3b7.png)  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/de50e9e11df3efa8d632fb93f7952cae.png)  
这时候我们再启动底盘的节点，并RVIZ中点击一下“2D Pose Estimate”就可以正常定位了

```
source / catkin_ws/devel/setup.bash
roslaunch p3at_bringup p3at_bringup.launch
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/38050c7589052f3ac0293fe5842ee942.png)  
接下来开始配置move\_base模块，move\_base模块需要有五个文件分别是：local\_costmap\_params.yaml（局部costmap参数文件）、global\_costmap\_params.yaml（全局地图参数文件）、dwa\_local\_planner\_params.yaml(局部规划器参数文件)、global\_planner\_params.yaml（全局规划器参数文件）、move\_base\_params.yaml（move\_base运行参数）。

其中xxx\_costmap\_params.yaml两个文件主要是配置地图的，比如地图的膨胀区域。xxx\_ planner\_params.yaml两个文件主要的规划器的运行参数。基本上这些参数按照wiki上的教程进行配置能运行起来，但是运行的不太好，这里面的参数很多还需要进一步理解。我在配置的时候主要参考了这两篇博客：[【1】](https://blog.csdn.net/qq_25241325/article/details/81207278)[【2】](https://blog.csdn.net/forrest_z/article/details/62891055)（后续有新的理解再更新到博客中）

在文件夹“p3at\_navigation/launch”中建立名为" amcl\_demo.launch"的文件，将所有的节点放在一个launch文件中启动：

```
<?xml version="1.0"?>
<launch>
  <arg name="rviz" default="true"/>

  <!-- Map Server -->
 <arg name="map_file" default="$(find p3at_navigation)/map/cqu_lab.yaml"/>
 <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

  <!-- AMCL -->
 <include file="$(find p3at_navigation)/launch/include/amcl.launch.xml" />

  <!-- Move Base -->
 <include file="$(find p3at_navigation)/launch/include/move_base.launch.xml" />

  <!-- RViz -->
  <node if="$(arg rviz)" pkg="rviz" type="rviz" name="$(anon rviz)" respawn="false" output="screen" args="-d $(find p3at_navigation)/rviz/localization.rviz" />

</launch>
```

配置好move\_base以后，我们将前面打开的终端都关闭，新开启两个终端，第一个终端启动p3at底盘已经传感器

```
source / catkin_ws/devel/setup.bash
roslaunch p3at_bringup p3at_bringup.launch
```

再开启一个终端启动move\_base 、map\_server以及amcl

```
source / catkin_ws/devel/setup.bash
roslaunch p3at_navigation amcl_demo.launch
```

欢迎留言讨论或 cenruping@vip.qq.com  
代码见GitHub：https://github.com/RuPingCen/P3AT\_Auto\_Navigation

上一篇 ： [自主导航小车实践（二）](https://blog.csdn.net/crp997576280/article/details/99701706)