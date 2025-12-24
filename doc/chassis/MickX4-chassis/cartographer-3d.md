#### 开源自主导航小车MickX4（七）cartographer 室外3D建图

+   [1 cartographer 3D建图demo](#1_cartographer__3Ddemo_7)
+   +   [1.1 cartographer 安装](#11_cartographer___12)
    +   [1.2 3D数据集建图](#12_3D_14)
    +   [1.3 3D定位](#13_3D_82)
+   [2 小车上的3D建图](#2_3D_104)
+   +   [2.1 配置launch文件](#21_launch_105)
    +   [2.2 配置TF 关系](#22_TF__121)
    +   [2.3 配置lua参数文件](#23_lua_149)
+   [3 大范围场景测试](#3__222)
+   [3.1 路线1](#31_1_223)
+   [3.2 路线2](#32_2_233)
+   [4 小结](#4__245)
+   [参考资料](#_256)

首先，在前面我们已经实现了2D建图，为什么还要测试室外的3D建图？

+   前面的地图构建时候是通过压缩3D到2D，实际只取了16线激光雷达中的一条线，信息利用不充分，建图的效果也不理想。
+   3D转2D的建图在空旷的场景（空间大于20m）时2D点就比较稀疏了，会造成地图更新失败，小车无法计算自己的位姿的情况。
+   3D 转2D 以后在空旷或者非结构化的场景（室外的树）会造成定位不准。

## 1 cartographer 3D建图demo

3D建图的时候需要提供IMU的信息，这里我们使用的是Xsens MTi-30的IMU，XsensIMU的安装方法可以参考我们博客。  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/42412d879ada6354376e30c67504fe1f.png#pic_center)

### 1.1 cartographer 安装

cartographer 的安装可以参考我们[上一篇博客中的介绍](https://blog.csdn.net/crp997576280/article/details/109685590)。

### 1.2 3D数据集建图

使用3D激光雷达建图的时候我们必须要结合IMU，使用IMU提供的重力方向向量。这里我们直接根据官网\[1\] 的步骤进行运行，首先你需要去下载这个3D数据包\[5\]

其次我们需要将官网上的 “offline\_backpack\_3d.launch”替换为 “demo\_backpack\_3d.launch”,否则在保存地图的时候会出现无法调用

启动3D激光雷达建图

```bash
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=/media/crp/0E3C06880E3C0688/b3-2016-04-05-13-54-42.bag
```

等到数据运行完毕以后调用 write\_state 服务来保存地图

```bash
rosservice call /write_state ~/demo_3d_local.pbstream
```

将这个pbstream文件进一步转化成3D的ply点云文件（注意使用绝对路径）

```bash
roslaunch cartographer_ros assets_writer_backpack_3d.launch bag_filenames:=/media/crp/0E3C06880E3C0688/b3-2016-04-05-13-54-42.bag pose_graph_filename:=/home/administrator/demo_3d_local.pbstream
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/91c746d702e86a8dd8aa442dad8df1f6.png#pic_center)

等待一段时间，处理完成后命令会自动退出，此时在bag文件旁边会生成一个.bag\_points.ply后缀文件，这个就是点云文件\[6\].最后利用PCL自带的工具将ply文件转换成pcd文件

```bash
pcl_ply2pcd b3-2016-04-05-13-54-42.bag_points.ply test_3d.pcd
```

在运行的时候，机器人的位姿是发布在TF中的，如下图所示。因此我们可以通过读取odom->map之间的坐标变换来知道机器人的位置  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/6768a82f07ca7bd93ea4ecda9f3290c0.png#pic_center)  
这个TF关系是通过 目录 “cartographer\_ros/cartographer\_ros/urdf/backpack\_3d.urdf” 这个urdf文件来进行配置的

以下是建图时候所使用的部分配置参数

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2, #使用两个点云
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}
```

### 1.3 3D定位

3D定位我们是利用在3.1部分生成的 “\*\*\*~/3d\_local.pbstream\*\*\*”作为已有地图，将当前激光数据输入进行匹配，估计位置

```bash
cartographer_ros demo_backpack_3d_localization.launch load_state_filename:=/home/crp/3d_local.pbstream bag_filename:=/media/crp/0E3C06880E3C0688/b3-2016-04-05-15-52-20.bag
```

同样在运行定位的时候，机器人的位姿也是发布在TF中的，如下图所示。因此我们可以通过读取odom->map之间的坐标变换来知道机器人的位置。（可以明显看出，定位时候的位姿输出频率要远远低于建图时候的频率）  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/a671f7f02d196c6e4fccd54984431dec.png)  
定位时候的参数（在建图参数上增加了两行）

```lua
include "backpack_3d.lua"

TRAJECTORY_BUILDER.pure_localization = true
POSE_GRAPH.optimize_every_n_nodes = 100

return options
```

## 2 小车上的3D建图

### 2.1 配置launch文件

新建文件 demo\_mickx4\_3d.launch ，填入以下内容：

```xml
<launch>
  <param name="/use_sim_time" value="true" />
  
  <include file="$(find cartographer_ros)/launch/mickx4_mapping_3d.launch" />

  <node name="rviz" pkg="rviz" type="rviz" required="true"
      args="-d $(find cartographer_ros)/configuration_files/mickx4_mapping_3d.rviz" />

  <node name="playbag" pkg="rosbag" type="play" args="--clock $(arg bag_filename)" />
</launch>
```

### 2.2 配置TF 关系

3D建图必须要加入IMU设备，IMU的坐标这里于demo的坐标系保持一致（发布imu\_link 到 base\_link之间的关系），这里我是用URDF 文件传入的。 新建文件 mickx4\_mapping\_3d.launch ，填入以下内容：

```html
<launch>

  <param name="robot_description" textfile="$(find cartographer_ros)/urdf/mickx4.urdf" />

  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
 
    <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename mickx4_mapping_3d.lua"
      output="screen">
    <remap from="points2" to="/rslidar_points" />
    <remap from="/imu" to="/imu/data" />
    <remap from="/odom" to="/odom" />
    
  </node>

  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node" args="-resolution 0.05" />

</launch>
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/a5574c43ef6a71599d8e08685188830e.png#pic_center)

### 2.3 配置lua参数文件

这里我发现四轮差速的里程计不准，也就没有使用里程计。有里程及计的话TF 树也是这样，只需要吧topic和参数（provide\_odom\_frame 和 use\_odometry）使能就可以了

```lua
include "map_builder.lua"
include "trajectory_builder.lua"


options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.2
TRAJECTORY_BUILDER_3D.max_range = 150
TRAJECTORY_BUILDER_2D.min_z = 0.1
TRAJECTORY_BUILDER_2D.max_z = 1.0
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 20
POSE_GRAPH.constraint_builder.min_score = 0.5
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55


POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

return options
```

启动 cartographer 建图节点

```bash
roslaunch cartographer_ros demo_mickx4_3d.launch bag_filename:=/home/administrator/bagfiles/mick_2021-01-27-21-34-13.bag
```

下面是在实验室门口外运行的视频，场景属于室外小范围场景

Cartographer 在机器人上实现3D建图-小场景

等到数据运行完毕以后调用 write\_state 服务来保存地图

```bash
rosservice call /write_state ~/mickx4_3d_mapping.pbstream
```

修改以后的 cartographer\_ros和相关的配置参数 可以在[我的github \[7\]](https://github.com/RuPingCen/cartographer-modify) 上找到。

## 3 大范围场景测试

## 3.1 路线1

第一个场景是围绕我们学校的主教楼转了一圈，大约有1公里，最后回环没有回上，我只保留了效果较好的一部分地图。地图在有结构化的场景中建图效果是比较好的，在操场边那个地方停有很多的车，其环境比较空旷。在这种情况下感觉估计的位姿就不准了，导致后面的点云有重影

```bash
roslaunch cartographer_ros demo_mickx4_3d.launch bag_filename:=/home/administrator/bagfiles/MickX4_Dataset/mick_2021-02-01-15-43-14.bag 
```

cartographer-室外-3D-大场景建图-1KM

## 3.2 路线2

第一个场景是围绕我们学校的几个楼之间转了一圈，大约有1.5公里。这个场景中大部分都是沿着公路走的，公路旁边都是建筑物和数，整体的定位效果很好，走一圈下来位置也没差多远，估计有个2M左右。

```bash
roslaunch cartographer_ros demo_mickx4_3d.launch bag_filename:=/home/administrator/bagfiles/MickX4_Dataset/mick_2021-02-01-15-22-28.bag 
```

cartographer-室外-3D-大范围场景-1.5KM

有需要 大范围场景测试 中的两个录制的bag的同学，可在评论区留下邮箱。（记得来一波点赞加关注哦）

## 4 小结

cartographer 3D 建图的效果不是很理想，总体而言存在很多的噪点。但是3D建图的效果比2D要好的多，主要是2D是利用3D进行压缩得到的2D平面，室外毕竟不是规则的环境，这个鲁棒性还是要弱一些，因此在室外使用2D激光还是不是太好。测试中还发现：

1）cartographer 3D 对环境比较敏感（校园里面树木比较多，一般都长在路面两侧，会建筑物的墙面有遮挡，导致结构化的元素比较少。因此在这段区域建图的效果就不是很好。等到了后面的路线中，没有树木了，激光可以扫描到两边的墙体时候建图的效果就好了很多了）  
2）对地面也比较敏感，在中间的一段，地面有坑，导致小车会上下颠簸，这种因素也会对建图产生影响。  
3）激光打到地面，carto 没有去地面的操作，因此需要我们自己调节激光发射角度，调整激光打到地面上的程度

效果比起LOAM系列的框架还是有点差距。后续可以尝试使用LOAM系列的框架进行测试  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/0c3e8a040bf1d6364731b43321bb4b95.png#pic_center)

此外，[蓝鲸机器人博客\[8\]](https://community.bwbot.org/topic/523/%E8%B0%B7%E6%AD%8Ccartographer%E4%BD%BF%E7%94%A8%E9%80%9F%E8%85%BE%E8%81%9A%E5%88%9B3d%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E6%95%B0%E6%8D%AE%E8%BF%9B%E8%A1%8C%E4%B8%89%E7%BB%B4%E5%BB%BA%E5%9B%BE)上的资料很详细，对cartography 的演示demo也完全可以复现，因此如果跑不起来也可以参考以下他们的资料\[6\]。

## 参考资料

\[1\] https://www.cnblogs.com/hitcm/p/5939507.html  
\[2\] https://blog.csdn.net/crp997576280/article/details/103279649  
\[3\] https://github.com/cartographer-project/cartographer\_ros  
\[4\] https://github.com/cartographer-project/cartographer  
\[5\] https://github.com/ceres-solver/ceres-solver  
\[6\] https://github.com/BluewhaleRobot/cartographer\_ros  
\[7\] https://github.com/RuPingCen/cartographer-modify  
\[8\] https://community.bwbot.org/topic/523/%E8%B0%B7%E6%AD%8Ccartographer%E4%BD%BF%E7%94%A8%E9%80%9F%E8%85%BE%E8%81%9A%E5%88%9B3d%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E6%95%B0%E6%8D%AE%E8%BF%9B%E8%A1%8C%E4%B8%89%E7%BB%B4%E5%BB%BA%E5%9B%BE

上一篇：[开源自主导航小车MickX4（六）cartographer 室外2D建图](https://blog.csdn.net/crp997576280/article/details/109685590)

下一篇：[开源自主导航小车MickX4（八）LeGo-LOAM 室外3D建图](https://blog.csdn.net/crp997576280/article/details/111657554)

**欢迎大家点赞在评论区交流讨论（cenruping@vip.qq.com） O(∩\_∩)O**

或者加群交流（1149897304）  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/32fa6571126eb1eba4d079c3496b87d6.png)