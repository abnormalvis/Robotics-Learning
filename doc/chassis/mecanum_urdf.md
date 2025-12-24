#### 开源自主导航小车MickX4（四）底盘URDF模型

- [1、底盘URDF模型](#1底盘urdf模型)
  - [1.1 URFD语法](#11-urfd语法)
  - [1.2 基于URDF语法绘制机器人](#12-基于urdf语法绘制机器人)
  - [1.3 URDF文件加载stl模型](#13-urdf文件加载stl模型)
- [2、绘制机器人URDF模型](#2绘制机器人urdf模型)
- [参考资料](#参考资料)

## 1、底盘URDF模型

URDF（Unified Robot Description Format）是ROS中描述机器人外形尺寸的一种语言，翻译的名称为“机器人统一描述”。 也就是说在RVIZ中显示一个3D的机器人模型，这个模型具备有一些模块具备TF坐标系关系，告诉系统机器人的实际尺寸，这个在导航中会涉及到。

创建功能包

```bash
catkin_create_pkg mick_description urdf
```

这里有两种方法加载机器人模型，第一种就是通过URDF描述的规则绘制一个机器人模型，另外一种就是先利用3D绘图软件绘制机器人3D模型文件，如stl文件，然后通过URDF描述语言直接加载stl文件。

### 1.1 URFD语法

```xml
<link>对应模型的一个模块，可以通过标签joint让子模块与base_link进行关联;

<visual>描述一个link的外观，大小，颜色，材质纹理等;

<geometry>定义该link的几何模型，包含该几何模型的尺寸，单位：米；

<box> 矩形，定义属性：size（包含长宽高，数据用空格隔开）；

<cylinder>  圆柱体，定义属性：length，radius；

<sphere>  球体，定义属性：radius；

<material> 定义颜色和透明度（RGBA），取值区间 [0,1] ；

<collision> 描述碰撞检测属性；

<origin> 用来描述模块的位置；

<inertial> 定义惯性；
```

如下图所示，URDF主要涉及到的有 **link** 和 **joint** 两类（为了表示严谨，这里主要是指基本的一些组件）。 **link** 主要是用来描述一个“零件” 的几何尺寸、颜色、材料属性等特征，而 **joint** 则是描述两个“零件”是一个怎么样的连接关系，相对坐标信息。下图总结了urdf中常用的标签，参考了博客[\[5\]](https://blog.csdn.net/shenyan0712/article/details/89919959)  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/8b92fbd5f22cf5e2908703fb77c5cec5.png#pic_center)

### 1.2 基于URDF语法绘制机器人

从上面可以看到urdf语言可以描述关节以及关节所联系物体的集合形状，用上面的语法可以把机器人用基本的原型和正方形把机器人给拼起来。

**step1:** 创建名为 **mick\_description** 的ROS包并在其中建立子文件夹 launch 和 urdf。

**step2:** 在文件夹下新建文件 “test.urdf” ，写入以下内容（下面的代码参考\[1\]）：

```xml
<?xml version="1.0"?>
<robot name="smartcar">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.25 .16 .05"/>
    </geometry>
    <origin rpy="0 0 0" xyz="0 0 0.0"/>
    <material name="gree">
        <color rgba="0 0 .8 1"/>
    </material>
    </visual>
 </link>

 <link name="left_front_wheel">
    <visual>
      <geometry>
        <cylinder length=".02" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_front_wheel_joint" type="continuous">
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="right_front_wheel"/>
    <origin rpy="1.57075 0 0" xyz="0.12 0.1 0"/>
    <limit effort="100" velocity="100"/>
    <joint_properties damping="0.0" friction="0.0"/>
  </joint>
 <link name="right_front_wheel">
    <visual>
      <geometry>
        <cylinder length=".02" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="right_front_wheel_joint" type="continuous">
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="left_front_wheel"/>
    <origin rpy="1.57075 0 0" xyz="0.12 -0.1 0"/>
    <limit effort="100" velocity="100"/>
    <joint_properties damping="0.0" friction="0.0"/>
  </joint>
  <link name="right_back_wheel">
    <visual>
      <geometry>
        <cylinder length=".02" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="right_back_wheel_joint" type="continuous">
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="right_back_wheel"/>
    <origin rpy="1.57075 0 0" xyz="-0.12 -0.1 0"/>
    <limit effort="100" velocity="100"/>
    <joint_properties damping="0.0" friction="0.0"/>
 </joint>

  <link name="left_back_wheel">
    <visual>
      <geometry>
        <cylinder length=".02" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_back_wheel_joint" type="continuous">
    <axis xyz="0 0 1"/>
    <parent link="base_link"/>
    <child link="left_back_wheel"/>
    <origin rpy="1.57075 0 0" xyz="-0.12 0.1 0"/>
    <limit effort="100" velocity="100"/>
    <joint_properties damping="0.0" friction="0.0"/>
  </joint>

  <link name="camera">
    <visual>
      <geometry>
        <box size=".02 .03 .03"/>
      </geometry>
      <material name="white">
          <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera"/>
    <origin xyz="0.1 0.0 0.025"/>
  </joint>
</robot>
```

这里我们使用 **link** 这个标签描述了四个半径为0.25m 厚度为0.02 的圆形轮子（**left\_front\_wheel，right\_front\_wheel，left\_back\_wheel，right\_back\_wheel**） ，然后通过标签 **joint** 把四个轮子和base\_link联系起来，并确定相对于base\_link的坐标关系。

接下来在当前目录下新建终端，使用命令检查urdf语法是否正确

```bash
check_urdf test.urdf
```

通过生成图形化的的URDF检查你的配置是否正确,这条命令会在目录下生成一个pdf文件，打开以后即可看到如下的TF树形关系。

```bash
 urdf_to_graphiz test.urdf
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/20e81bdc42af0227112cc5c54c24be44.png#pic_center)

语法检查通过以后我们可以新建一个 display.launch 文件填入以下内容

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>  
  <arg name="model" /> 
  <arg name="gui" default="false" />
  <arg name="rvizconfig" default="$(find mick_description)/rviz/view_modle.rviz" /> 
 
  <param name="robot_description" textfile="$(find mick_description)/urdf/test.urdf" />
  <!-- param name="robot_description" command="$(find xacro)/xacro.py $(find mick_description)/urdf/p3at.urdf.xacro" /-->  
  <param name="use_gui" value="$(arg gui)"/>  

  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" /> 
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" /> 
 
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />  

</launch> 
```

启动launch文件以后可以看到如下效果的一个小车  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/d663e0e41aa39bb6945c0cb58f532005.png#pic_center)

### 1.3 URDF文件加载stl模型

理论上我们利用基本的球体、圆柱体和长方体可以组合成任意形状的小车，但是这针对一个复杂的车系统就比较麻烦了，而且一般在设计机结构的时候都会由机械工程师出3D图和加工图纸。

如果你自己绘制了一个小车的3D文件，保存为了 xxx.stl 模型，则可以通过加载现有的stl模型实现urdf模型。这里需要在我们创建功能包中 **urdf** 文件夹创建子 **mesh** 文件夹。在\[2\] 中下载autolabor的stl 3D文件到mesh文件夹中。

新建 test\_mesh.urdf 文件， 填入以下内容可得

```xml
 <robot name="autolabor"> 
    <link name="base_link">
    <inertial>
      <origin
        xyz="0. 0. 0."
        rpy="0. 0. 0." />
      <mass
        value="0.251988675650349" />
      <inertia
        ixx="0.000595579869264794"
        ixy="5.99238175321912E-08"
        ixz="-1.98242615307314E-08"
        iyy="0.00102462329604677"
        iyz="-1.73115625503396E-05"
        izz="0.00060561972360446" />
    </inertial>
    <visual>
      <origin
        xyz="0. 0. 0.05"
        rpy="1.57 0. 1.57" />
      <geometry>
        <mesh
          filename="package://mick_description/urdf/mesh/autolabor.stl" />
      </geometry>
      <material
        name="">
        <color
          rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
      </material>
    </visual>
   </link>
</robot>
```

启动launch文件就可以看到加载到的 stl 文件了， 但是注意这里只描述了base\_link的3D 文件，没有描述其他的坐标系，比如说我们需要用的的传感器 laser 和 camera。这些还需要自行添加其他的 link 标签。

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/21243ba89fdfd1b0a3fc6a593567d439.png#pic_center)

## 2、绘制机器人URDF模型

这里由于自己不会3D作图，因此使用了车体我通过方块进行拼接，部分传感器通过联系厂家找到stl文件添加到车体上。由于这个urdf文件比较长，因此[详细代码可以参考链接\[3\]](https://github.com/RuPingCen/mick_robot/blob/master/mick_description/urdf/mickX4.urdf)：

最终通过launch文件启动以后，可以看到如下效果，大体上还是比较像真实用的小车的

```bash
roslaunch mick_description display.launch
```

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/621e84f026ef8b9462c9dec7023d1653.png#pic_center)  
看起来还是有点像，[完整的节点包\[4\]](https://github.com/RuPingCen/mick_robot/tree/master/mick_description)

## 参考资料

\[1\] https://www.jianshu.com/p/b8269e8269ba  
\[2\] http://www.autolabor.com.cn/download?hmsr=jianshu  
\[3\] https://github.com/RuPingCen/mick\_robot/blob/master/mick\_description/urdf/mickX4.urdf  
\[4\] https://github.com/RuPingCen/mick\_robot/tree/master/mick\_description  
\[5\] https://blog.csdn.net/shenyan0712/article/details/89919959

上一篇：[开源自主导航小车MickX4（三）底盘ROS节点](https://blog.csdn.net/crp997576280/article/details/108567732)

**欢迎大家点赞在评论区交流讨论（cenruping@vip.qq.com） O(∩\_∩)O**

或者加群交流（1149897304）  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/e7e3b9b172150d7e0831c2b41f147adc.png#pic_center)