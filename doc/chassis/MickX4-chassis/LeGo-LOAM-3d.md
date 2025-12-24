#### 开源自主导航小车MickX4（八）LeGo-LOAM 室外3D建图

- [1 安装 LeGO-LOAM](#1-安装-lego-loam)
  - [1.1安装依赖项](#11安装依赖项)
- [2 运行 LeGO-LOAM](#2-运行-lego-loam)
  - [2.1 运行论文demo](#21-运行论文demo)
  - [2.2 修改配置文件](#22-修改配置文件)
  - [2.3 运行自己的bag包](#23-运行自己的bag包)
  - [2.4 地图保存](#24-地图保存)
- [3 大范围场景测试](#3-大范围场景测试)
- [3.1 路线1](#31-路线1)
- [3.2 路线2](#32-路线2)
- [4 小结](#4-小结)
- [参考资料](#参考资料)

## 1 安装 LeGO-LOAM

LeGO-LOAM \[1\] 需要依赖 ROS 环境 和 gtsam

### 1.1安装依赖项

**step1** 安装 gtsam

```bash
wget wget -O ~/software/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
cd ~/software && unzip gtsam.zip -d ~/software/
cd ~/software/gtsam-4.0.0-alpha2/
mkdir build && cd build
cmake ..
sudo make install
```

**step2** 安装 下载编译源码

```bash
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
cd ..
catkin_make -j1

```

这两步基本没有很大的问题，按照作者给出的指令即可安装成功。

## 2 运行 LeGO-LOAM

### 2.1 运行论文demo

LeGO-LOAM 坐在提供了一些bag包，可以通过这些bag包检测我们安装的环境是否正确，还可以查看以下 LeGO-LOAM 的建图效果。  
**step1** 启动 LeGO-LOAM

```bash
roslaunch lego_loam run.launch
```

**step2** 播放bag包， bag可以在作者的github上\[1\]下载（可能会很慢），也可以在我们的百度云里面下载。

```bash
rosbag play lego-loam-demo-2017-06-08-15-49-45_0.bag --clock --topic /velodyne_points /imu/data
```

### 2.2 修改配置文件

在使用速腾16线类雷达的时候注意修改 launch 文件 和 utility.h 配置文件。 LeGO-LOAM 不需要里程计 且 IMU 也不是必须的。  
**step1** 创建 launch 文件 **mickx4.launch**

```xml
<launch>
    
    <!--- Sim Time -->
    <param name="/use_sim_time" value="true" />
 
    <node pkg="nodelet" type="nodelet" name="pcl_manager" args="manager" output="screen" />
    
    <!-- Run a passthrough filter to clean NaNs -->
    <node pkg="nodelet" type="nodelet" name="passthrough" args="load pcl/PassThrough pcl_manager" output="screen">
        <remap from="~input" to="/rslidar_points" />
        <remap from="/passthrough/output" to="/velodyne_points" />
        
        <rosparam>
        filter_field_name: z
        filter_limit_negative: True
        </rosparam>

    <rosparam>
        filter_field_name: x
        filter_limit_negative: True
        </rosparam>

    <rosparam>
        filter_field_name: y
        filter_limit_negative: True
       
        </rosparam>
    </node>

    <!--- Run Rviz-->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find lego_loam)/launch/test.rviz" />

    <!--- TF -->
    <node pkg="tf" type="static_transform_publisher" name="camera_init_to_map"  args="0 0 0 1.570795   0        1.570795 /map    /camera_init 10" />
    <node pkg="tf" type="static_transform_publisher" name="base_link_to_camera" args="0 0 0 -1.570795 -1.570795 0        /camera /base_link   10" />

 
    <!--- LeGO-LOAM -->    
    <node pkg="lego_loam" type="imageProjection"    name="imageProjection"    output="screen"/>
    <node pkg="lego_loam" type="featureAssociation" name="featureAssociation" output="screen"/>
    <node pkg="lego_loam" type="mapOptmization"     name="mapOptmization"     output="screen"/>
    <node pkg="lego_loam" type="transformFusion"    name="transformFusion"    output="screen"/>

</launch>

```

中间的 “ Run a passthrough filter to clean NaNs ”段是为了去除 速腾16线的激光雷达的 NaN 点\[2\] 。

**step2** 修改 **utility.h** 文件

在LeGO-LOAM/LeGO-LOAM/include文件夹的 utility.h 文件中, **useCloudRing** false

```cpp
extern const bool useCloudRing = false;
```

因为Velodyne的雷达单独有一个ring通道 \[3\]，而robosense的雷达好像并没有,在代码中可以将这一功能关闭。如不修改启动的时候会报错 测试时报错:Failed to find match for field ‘ring’

运行作者提供的数据包的视频（作者提供的额数据包在github上可以下载，也可以在这个[链接里面下载](https://pan.baidu.com/s/1Zmhu-xKH-kyLn_br7BhQ4w) （提取码：yd1o ））

### 2.3 运行自己的bag包

启动 LeGO-LOAM

```bash
roslaunch lego_loam mickx4.launch
```

运行bag ,我们自己录制的bag可以在这个[链接里面下载](https://pan.baidu.com/s/1Zmhu-xKH-kyLn_br7BhQ4w) （提取码：yd1o ）

```bash
rosbag play mick_2021-01-28-10-03-01.bag --clock --topic /rslidar_points /imu/data
```

点云输出在 **/laser\_cloud\_surround** Topic上 PointCloud2类型， 可以通过第三方脚本进行保存。

下面是在实验室门口外运行的视频，场景属于室外小范围场景

LeGO-LOAM-3D mapping owith MickX4 -小范围场景

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/c3ddfaa7d3af54fe0a64b075f3502c7f.png#pic_center)

### 2.4 地图保存

LeGO-LOAM 的点云地图是发布在 /laser\_cloud\_surround 这个话题上的，[在这篇博客\[7\]](https://blog.csdn.net/qq_36396941/article/details/83048415)中提供了一种方法就是记录这个topic上的数据，然后利用 pcl\_ros 包进行转换。  
**step1:** 记录话题 **/laser\_cloud\_surround** 数据,这个可以在快结束的时候录制

```bash
rosbag record -o lego-loam-out /laser_cloud_surround
```

**step2:** 把 lego-loam-out\* 这个bag的话题上的数据存为pcd文件

```bash
rosrun pcl_ros bag_to_pcd lego-loam-out_2021-02-01-17-01-06.bag /laser_cloud_surround test
```

这时候会在你的目录下生成一个test目录，然后里面会有很多个pcd文件，选最后一个就是最新的点云地图了

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/bc4acddab2c32445200ede9ef94de305.png#pic_center)

## 3 大范围场景测试

## 3.1 路线1

第一个场景是围绕我们学校的主教楼转了一圈，大约有1公里，最后回环没有回上，我只保留了效果较好的一部分地图。地图在有结构化的场景中建图效果是比较好的，在操场边那个地方停有很多的车，其环境比较空旷。在这种情况下感觉估计的位姿就不准了，导致后面的点云有重影

```bash
roslaunch lego_loam mickx4.launch
```

```bash
rosbag play mick_2021-02-01-15-43-14.bag --clock --topic /rslidar_points /imu/data
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/0028d8c63372d32040aecabb8cc71842.png#pic_center)

## 3.2 路线2

第一个场景是围绕我们学校的几个楼之间转了一圈，大约有1.5公里。这个场景中大部分都是沿着公路走的，公路旁边都是建筑物和数，整体的定位效果很好，走一圈下来位置也没差多远，估计有个2M左右。

```bash
roslaunch lego_loam mickx4.launch
```

```bash
rosbag play mick_2021-02-01-15-22-28.bag --clock --topic /rslidar_points /imu/data
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/dee94673ed3be1e9fef5e29e19a0eb69.jpeg#pic_center)

## 4 小结

这里运行的环境还是前几篇博客中的场景, 从我自己测试的结果来看 LeGO-LOAM 在 3D 环境下建图效果要优于 Cartographer。博客中使用的代码和launch文件可以在[Github\[8\].](https://github.com/RuPingCen/mick_robot/tree/master/SLAM/LeGO-LOAM/LeGO-LOAM)下载

关于 LeGO-LOAM 的计算原理和模型可以[参考博客\[4\]](https://blog.csdn.net/try_again_later/article/details/105367531) 。 此外LeGO-LOAM 还有一个升级版 [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)，效果比LeGO-LOAM更加好。

LeGO-LOAM 没有地图保存、重定位和回环检测的功能，地图保存这个比较简单，可以通过自己写一个节点保存点云，但是重定位的功能就不太容易了。

有需要 **大范围场景测试** 中的两个录制的bag的同学，可在评论区留下邮箱。（记得来一波点赞加关注哦）

## 参考资料

\[1\] https://github.com/RobustFieldAutonomyLab/LeGO-LOAM  
\[2\] https://blog.csdn.net/heirenlop/article/details/111475684  
\[3\] https://blog.csdn.net/weixin\_39754100/article/details/112186264  
\[4\] https://blog.csdn.net/try\_again\_later/article/details/105367531  
\[5\] https://arxiv.org/abs/2007.00258  
\[6\] https://github.com/TixiaoShan/LIO-SAM  
\[7\] https://blog.csdn.net/qq\_36396941/article/details/83048415  
\[8\] https://github.com/RuPingCen/mick\_robot/tree/master/SLAM/LeGO-LOAM/LeGO-LOAM

上一篇：[开源自主导航小车MickX4（七）cartographer 室外3D建图](https://blog.csdn.net/crp997576280/article/details/111600534)

下一篇：开源自主导航小车MickX4（九）基于move\_base 的自主导航框架

**欢迎大家点赞在评论区交流讨论（cenruping@vip.qq.com） O(∩\_∩)O**

或者加群交流（1149897304）  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/32fa6571126eb1eba4d079c3496b87d6.png)