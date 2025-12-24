**目录**

[1.底盘硬件框图](#1.%E5%BA%95%E7%9B%98%E7%A1%AC%E4%BB%B6%E6%A1%86%E5%9B%BE "1.底盘硬件框图")

[2、麦克纳姆轮模型介绍](#1%E3%80%81%E9%BA%A6%E5%85%8B%E7%BA%B3%E5%A7%86%E8%BD%AE%E4%BB%8B%E7%BB%8D "2、麦克纳姆轮模型介绍")

[2.1 麦克纳姆轮下的运动学模型](#2.1%E9%80%86%E8%A7%A3%E8%BF%90%E5%8A%A8%E5%AD%A6%E6%A8%A1%E5%9E%8B "2.1 麦克纳姆轮下的运动学模型")

[2.2 正解运动学模型](#2.2%E6%AD%A3%E8%A7%A3%E8%BF%90%E5%8A%A8%E5%AD%A6%E6%A8%A1%E5%9E%8B "2.2 正解运动学模型")

 [2.3 逆解运动学模型](#%C2%A02.3%C2%A0%E9%80%86%E8%A7%A3%E8%BF%90%E5%8A%A8%E5%AD%A6%E6%A8%A1%E5%9E%8B " 2.3 逆解运动学模型")

[3 底盘软件框架](#3%20%E5%BA%95%E7%9B%98%E8%BD%AF%E4%BB%B6%E6%A1%86%E6%9E%B6 "3 底盘软件框架")

[4 参考资料](#5%20%E5%8F%82%E8%80%83%E8%B5%84%E6%96%99%EF%BC%9A "4 参考资料")

* * *

在这一个专题中，我们将介绍如何搭建利用麦克纳姆轮搭建一个ROS底盘。参考标准的ROS底盘，ROS底盘应该具备接收导航节点发送的“cmd\_vel”这个话题上的控制命令（小车X,Y,Z三个方向的速度以及三个方向的角速度），同时更加底盘轮子的转速信息预估底盘的位置，并发布到Odometry(里程计)话题上。

整个专题分为了四个部分实现：

      1、首先介绍小车的硬件结构，

     2、其次介绍麦克纳姆轮的运动学模型，

     3、接下来利用遥控器控制麦克纳姆轮前后左右移动实现遥控，

     4、最后编写ROS节点将“cmd\_vel”话题的数据转到底盘上。

所有的代码均放在了[Github](https://github.com/RuPingCen/mick_robot_chasiss "Github")中,欢迎大家下载留言。

## 1.底盘硬件框图

底盘的主要任务是接收上位机的指令（即ROS节点的指令），控制每个电机的转速、同时上传底盘上的IMU、超声波、电机转速等传感器和执行器的信息到ROS中的底盘节点。这里我们涉及到传感器和执行器分别有：2.4G遥控器接收机、IMU、超声波传感器、4路大疆M3508电机及驱动器。其中遥控器用于辅助遥控底盘和切换控制方式、超声波用于弥补激光雷达的盲区，IMU用于测量底盘的姿态和航向角（在有些底盘中还使用了IMU的信息来做了转角控制的闭环）

![](https://i-blog.csdnimg.cn/blog_migrate/185f9ce0f154bf39a3e8d6c502e63bc3.png)

                                                                                        图1-1 小车硬件框图

小车硬件实物如下图所示，定义左上角电机的编号为#1，左下为#2，右下为#3，右上为#4。正前方为X方向，水平方向为Y方向（与ROS中的坐标系相同）。

![](https://i-blog.csdnimg.cn/blog_migrate/8d3e5a4bd8a5d0f39d672a89062b9d2f.png)

                                                                                        图1-2 小车硬件实物

底盘的开发任务分四步实现：

     第一步，打通通讯链路，读取传感器（DBUS接收、IMU数据读取、超声波数据读取）和电机驱动器（C620）数据。

     第二步：利用PI控制器实现电机的速度闭环控制。

     第三步：利用麦克纳姆轮速度逆解模型实现X/Y/W三个方向的速度控制。

     第四步：完善上位机收发指令协议、上传IMU、超声波、电机转速信息。

接下来我们先介绍一下麦克纳姆轮的运动学模型，这个运动学模型的主要是方便我们在X,Y,W三个方向的速度和四个电机的转速（V1,V2,V3,V4）之间建立映射关系。控制的时候导航模块给定的是X，Y方向的速度（m/s）以及小车的旋转速度W（rad/s）,在发布里程计的时候，我们需要读取四个电机的转速或者转动的圈数来推算出小车的位置坐标和航向角。

## 2、麦克纳姆轮模型介绍

首先我们先从宏观的角度来分析一下麦克纳姆轮底盘是如何实现前后、左右、旋转运动的。

 

![](https://i-blog.csdnimg.cn/blog_migrate/0511f57eafa042b37c6f391756481f48.png)![](https://i-blog.csdnimg.cn/blog_migrate/44a895b6c454f5c703d09157a454bf6d.png)  
                                   a. 前后运动                                                               b. 左右运动  
 ![](https://i-blog.csdnimg.cn/blog_migrate/150d2ed56f745af877070431ed9d0bb5.png)![](https://i-blog.csdnimg.cn/blog_migrate/01b5a3a88da5b31cbf28d641d5b9cafd.png)

                                     c. 45度斜向运动                                                     d. 旋转运动

                                                        图2-1 麦克纳姆轮运动学模型示意图

麦克纳姆轮安装方式为“米”字形安装，如图1所示，假设我们需要控制小车动以下运动：

1.  向前运动，那么需要四个轮子向前转动
2.  向左运动时，需要左前和右后轮向后转动，左后和右前轮向前转动
3.  向右前方运动时，左前和右前轮向前转动
4.  逆时针旋转时，左前和左后向后转动，右前和右后向前转动

在参考\[1\]博客中已经给出了一个讲解麦克纳姆轮的经典教程，但是美中不足的是这个教程中的图片不知道怎么缘故无法显示，因此在参考这个大疆官网上的教程基础上进行了整理。

首先定义小车左右为Y方向，其中向左为正；前后为X方向，其中正前方为正；小车逆时针旋转为正用![{w_v}](https://private.codecogs.com/gif.latex?%7Bw_v%7D)表示。假设小车速度为![V=[V_x V_y V_z]](https://latex.csdn.net/eq?V%3D%5BV_x%20V_y%20V_z%5D)， 麦克纳姆轮从左上角到右上角标号依次为N=1,2,3,4。(Xn,Yn)为第n个轮相对原点的坐标，速度用![{​{\rm{v}}_n}{\rm{ = [}}{​{\rm{v}}_{xn}}\;{​{\rm{v}}_{yn}}{\rm{ }}\;{w_n}{\rm{]}}](https://private.codecogs.com/gif.latex?%7B%7B%5Crm%7Bv%7D%7D_n%7D%7B%5Crm%7B%20%3D%20%5B%7D%7D%7B%7B%5Crm%7Bv%7D%7D_%7Bxn%7D%7D%5C%3B%7B%7B%5Crm%7Bv%7D%7D_%7Byn%7D%7D%7B%5Crm%7B%20%7D%7D%5C%3B%7Bw_n%7D%7B%5Crm%7B%5D%7D%7D)表示。

各麦轮轮毂与X轴平行安装（通常呈米自形安装），r为轮毂半径，以逆时针方向偏转为正。![\theta _N](https://private.codecogs.com/gif.latex?%5Ctheta%20_N)为麦轮n的接地辊子轴线与X轴间的夹角【即辊子的偏置安装角，大小为45°】。辊子就是在麦克纳姆轮上斜着的那个。4个轮子的转速分别为![{​{\rm{v}}_n}{\rm{ = [}}{​{\rm{v}}_{xn}}\;{​{\rm{v}}_{yn}}{\rm{ }}\;{w_n}{\rm{]}}](https://private.codecogs.com/gif.latex?%7B%7B%5Crm%7Bv%7D%7D_n%7D%7B%5Crm%7B%20%3D%20%5B%7D%7D%7B%7B%5Crm%7Bv%7D%7D_%7Bxn%7D%7D%5C%3B%7B%7B%5Crm%7Bv%7D%7D_%7Byn%7D%7D%7B%5Crm%7B%20%7D%7D%5C%3B%7Bw_n%7D%7B%5Crm%7B%5D%7D%7D)。定义好这些以后我们开始推导麦克纳姆轮的运动学模型。

![](https://i-blog.csdnimg.cn/blog_migrate/3db71eb614a7c335ea654750b6ed9122.png)

                                                                                         图2-2 小车硬件框图

### 2.1 麦克纳姆轮下的运动学模型

已知四个轮子的转速![\Omega {\rm{ = [}}{w_1},{w_2},{w_3},{w_4}{​{\rm{]}}^T}](https://private.codecogs.com/gif.latex?%5COmega%20%7B%5Crm%7B%20%3D%20%5B%7D%7D%7Bw_1%7D%2C%7Bw_2%7D%2C%7Bw_3%7D%2C%7Bw_4%7D%7B%7B%5Crm%7B%5D%7D%7D%5ET%7D)求解车的速度![V=[V_x V_y V_z]](https://latex.csdn.net/eq?V%3D%5BV_x%20V_y%20V_z%5D)，也就是找到一个4x3矩阵R使得其满足：

                                            ![\Omega {\rm{ = }}\frac{1}{​{\rm{r}}}RV](https://private.codecogs.com/gif.latex?%5COmega%20%7B%5Crm%7B%20%3D%20%7D%7D%5Cfrac%7B1%7D%7B%7B%5Crm%7Br%7D%7D%7DRV)                                                                                   （2.1）

假设车身为刚体，每个麦轮的中心的水平和垂直速度分量![{​{\rm{v}}_n}{\rm{ = [}}{​{\rm{v}}_{xn}}\;{​{\rm{v}}_{yn}}{\rm{]}}](https://private.codecogs.com/gif.latex?%7B%7B%5Crm%7Bv%7D%7D_n%7D%7B%5Crm%7B%20%3D%20%5B%7D%7D%7B%7B%5Crm%7Bv%7D%7D_%7Bxn%7D%7D%5C%3B%7B%7B%5Crm%7Bv%7D%7D_%7Byn%7D%7D%7B%5Crm%7B%5D%7D%7D)均等于车的水平和垂直速度分量![V_n=[V_{xn} V_{yn}]](https://latex.csdn.net/eq?V_n%3D%5BV_%7Bxn%7D%20V_%7Byn%7D%5D)。此外，由于车身还有旋转速度![{w_v}](https://private.codecogs.com/gif.latex?%7Bw_v%7D), ![{w_v}](https://private.codecogs.com/gif.latex?%7Bw_v%7D)也会在每个麦轮上产生的（水平和垂直方向）的速度分量![{​{\rm{v}}_n}{\rm{ = [}}{​{\rm{v}}_{xn}}\;{​{\rm{v}}_{yn}}{\rm{]}}](https://private.codecogs.com/gif.latex?%7B%7B%5Crm%7Bv%7D%7D_n%7D%7B%5Crm%7B%20%3D%20%5B%7D%7D%7B%7B%5Crm%7Bv%7D%7D_%7Bxn%7D%7D%5C%3B%7B%7B%5Crm%7Bv%7D%7D_%7Byn%7D%7D%7B%5Crm%7B%5D%7D%7D) 

                                             ![\begin{array}{l} {​{\rm{v}}_{xrn}} = {\rm{ - Yn}} \cdot {w_n}\\ {​{\rm{v}}_{yrn}} = X{\rm{n}} \cdot {w_n} \end{array}](https://private.codecogs.com/gif.latex?%5Cbegin%7Barray%7D%7Bl%7D%20%7B%7B%5Crm%7Bv%7D%7D_%7Bxrn%7D%7D%20%3D%20%7B%5Crm%7B%20-%20Yn%7D%7D%20%5Ccdot%20%7Bw_n%7D%5C%5C%20%7B%7B%5Crm%7Bv%7D%7D_%7Byrn%7D%7D%20%3D%20X%7B%5Crm%7Bn%7D%7D%20%5Ccdot%20%7Bw_n%7D%20%5Cend%7Barray%7D)                                                                   （2.2）

因此综合而言每个麦轮在水平和垂直方向的速度分量为：

                                             ![\begin{array}{l} {​{\rm{v}}_{xn}}{\rm{ = }}{​{\rm{v}}_x}{\rm{ + }}{​{\rm{v}}_{xrn}} = {​{\rm{v}}_x}{\rm{ - }}{​{\rm{Y}}_n} \cdot {w_n}\\ {​{\rm{v}}_{yn}}{\rm{ = }}{​{\rm{v}}_y}{\rm{ + }}{​{\rm{v}}_{yrn}} = {​{\rm{v}}_y}{\rm{ + }}{X_n} \cdot {w_n} \end{array}](https://private.codecogs.com/gif.latex?%5Cbegin%7Barray%7D%7Bl%7D%20%7B%7B%5Crm%7Bv%7D%7D_%7Bxn%7D%7D%7B%5Crm%7B%20%3D%20%7D%7D%7B%7B%5Crm%7Bv%7D%7D_x%7D%7B%5Crm%7B%20&plus;%20%7D%7D%7B%7B%5Crm%7Bv%7D%7D_%7Bxrn%7D%7D%20%3D%20%7B%7B%5Crm%7Bv%7D%7D_x%7D%7B%5Crm%7B%20-%20%7D%7D%7B%7B%5Crm%7BY%7D%7D_n%7D%20%5Ccdot%20%7Bw_n%7D%5C%5C%20%7B%7B%5Crm%7Bv%7D%7D_%7Byn%7D%7D%7B%5Crm%7B%20%3D%20%7D%7D%7B%7B%5Crm%7Bv%7D%7D_y%7D%7B%5Crm%7B%20&plus;%20%7D%7D%7B%7B%5Crm%7Bv%7D%7D_%7Byrn%7D%7D%20%3D%20%7B%7B%5Crm%7Bv%7D%7D_y%7D%7B%5Crm%7B%20&plus;%20%7D%7D%7BX_n%7D%20%5Ccdot%20%7Bw_n%7D%20%5Cend%7Barray%7D)                                               （2.3）

到这我们求出了车身速度![V=[V_x V_y V_z]](https://latex.csdn.net/eq?V%3D%5BV_x%20V_y%20V_z%5D)对应到每个麦轮上的速度![{​{\rm{v}}_n}{\rm{ = [}}{​{\rm{v}}_{xn}}\;{​{\rm{v}}_{yn}}{\rm{]}}](https://private.codecogs.com/gif.latex?%7B%7B%5Crm%7Bv%7D%7D_n%7D%7B%5Crm%7B%20%3D%20%5B%7D%7D%7B%7B%5Crm%7Bv%7D%7D_%7Bxn%7D%7D%5C%3B%7B%7B%5Crm%7Bv%7D%7D_%7Byn%7D%7D%7B%5Crm%7B%5D%7D%7D)的关系，接下来只要找到每个轮子转速![{w_n}](https://private.codecogs.com/gif.latex?%7Bw_n%7D)与![{v_n}](https://private.codecogs.com/gif.latex?%7Bv_n%7D)之间的关系就构建完毕了。下图为右上轮在某一时刻的速度矢量关系。

![](https://i-blog.csdnimg.cn/blog_migrate/86a6a490f8ab71bab72a6bb2c9da1acc.png)

                                                                                        图2-3 小车硬件框图

 图2-3中Vn这个合速度矢量就是麦轮n当前的实际运动速度，由速度分量Vxn与Vyn合成。在图中还可以看出看出Vn是由麦轮n的轮毂中心速度Aî=ωn·r和接地自由辊子的中心速度Bû合成而来的（自由辊子的中心速度B是随速度矢量Vn变化的）。所以有等式：

                                               ![{w_n}r = {​{\rm{v}}_{xn}}{\rm{ + }}{​{\rm{v}}_{yn}}*\tan {\theta _n}](https://private.codecogs.com/gif.latex?%7Bw_n%7Dr%20%3D%20%7B%7B%5Crm%7Bv%7D%7D_%7Bxn%7D%7D%7B%5Crm%7B%20&plus;%20%7D%7D%7B%7B%5Crm%7Bv%7D%7D_%7Byn%7D%7D*%5Ctan%20%7B%5Ctheta%20_n%7D)                                                     （2.4）

变形为：

                                               ![\begin{array}{l} {w_n} = \frac{1}{r}[({​{\rm{v}}_x}{\rm{ - }}{​{\rm{Y}}_n} \cdot {w_n}){\rm{ + }}({​{\rm{v}}_y}{\rm{ + }}{X_n} \cdot {w_n})\tan {\theta _n}]\\ \;\;\;\;\; = \frac{1}{r}[{​{\rm{v}}_x}{\rm{ + }}{​{\rm{v}}_y}\tan {\theta _n}{\rm{ + }}({X_n}\tan {\theta _n} - {​{\rm{Y}}_n}) \cdot {w_n}] \end{array}](https://private.codecogs.com/gif.latex?%5Cbegin%7Barray%7D%7Bl%7D%20%7Bw_n%7D%20%3D%20%5Cfrac%7B1%7D%7Br%7D%5B%28%7B%7B%5Crm%7Bv%7D%7D_x%7D%7B%5Crm%7B%20-%20%7D%7D%7B%7B%5Crm%7BY%7D%7D_n%7D%20%5Ccdot%20%7Bw_n%7D%29%7B%5Crm%7B%20&plus;%20%7D%7D%28%7B%7B%5Crm%7Bv%7D%7D_y%7D%7B%5Crm%7B%20&plus;%20%7D%7D%7BX_n%7D%20%5Ccdot%20%7Bw_n%7D%29%5Ctan%20%7B%5Ctheta%20_n%7D%5D%5C%5C%20%5C%3B%5C%3B%5C%3B%5C%3B%5C%3B%20%3D%20%5Cfrac%7B1%7D%7Br%7D%5B%7B%7B%5Crm%7Bv%7D%7D_x%7D%7B%5Crm%7B%20&plus;%20%7D%7D%7B%7B%5Crm%7Bv%7D%7D_y%7D%5Ctan%20%7B%5Ctheta%20_n%7D%7B%5Crm%7B%20&plus;%20%7D%7D%28%7BX_n%7D%5Ctan%20%7B%5Ctheta%20_n%7D%20-%20%7B%7B%5Crm%7BY%7D%7D_n%7D%29%20%5Ccdot%20%7Bw_n%7D%5D%20%5Cend%7Barray%7D)                 （2.5）

对于4个轮子有：

                                               ![\left[ {\begin{array}{*{20}{c}} {​{w_1}}\\ {​{w_2}}\\ {​{w_3}}\\ {​{w_4}} \end{array}} \right]{\rm{ = }}\frac{1}{​{\rm{r}}}\left[ {\begin{array}{*{20}{c}} 1&{\tan {\theta _1}}&{​{X_1}\tan {\theta _1} - {​{\rm{Y}}_1}}\\ 1&{\tan {\theta _2}}&{​{X_2}\tan {\theta _2} - {​{\rm{Y}}_2}}\\ 1&{\tan {\theta _3}}&{​{X_3}\tan {\theta _3} - {​{\rm{Y}}_3}}\\ 1&{\tan {\theta _4}}&{​{X_4}\tan {\theta _4} - {​{\rm{Y}}_4}} \end{array}} \right]\left[ {\begin{array}{*{20}{c}} {​{​{\rm{v}}_x}}\\ {​{​{\rm{v}}_y}}\\ {​{​{\rm{v}}_z}} \end{array}} \right]\;](https://private.codecogs.com/gif.latex?%5Cleft%5B%20%7B%5Cbegin%7Barray%7D%7B*%7B20%7D%7Bc%7D%7D%20%7B%7Bw_1%7D%7D%5C%5C%20%7B%7Bw_2%7D%7D%5C%5C%20%7B%7Bw_3%7D%7D%5C%5C%20%7B%7Bw_4%7D%7D%20%5Cend%7Barray%7D%7D%20%5Cright%5D%7B%5Crm%7B%20%3D%20%7D%7D%5Cfrac%7B1%7D%7B%7B%5Crm%7Br%7D%7D%7D%5Cleft%5B%20%7B%5Cbegin%7Barray%7D%7B*%7B20%7D%7Bc%7D%7D%201%26%7B%5Ctan%20%7B%5Ctheta%20_1%7D%7D%26%7B%7BX_1%7D%5Ctan%20%7B%5Ctheta%20_1%7D%20-%20%7B%7B%5Crm%7BY%7D%7D_1%7D%7D%5C%5C%201%26%7B%5Ctan%20%7B%5Ctheta%20_2%7D%7D%26%7B%7BX_2%7D%5Ctan%20%7B%5Ctheta%20_2%7D%20-%20%7B%7B%5Crm%7BY%7D%7D_2%7D%7D%5C%5C%201%26%7B%5Ctan%20%7B%5Ctheta%20_3%7D%7D%26%7B%7BX_3%7D%5Ctan%20%7B%5Ctheta%20_3%7D%20-%20%7B%7B%5Crm%7BY%7D%7D_3%7D%7D%5C%5C%201%26%7B%5Ctan%20%7B%5Ctheta%20_4%7D%7D%26%7B%7BX_4%7D%5Ctan%20%7B%5Ctheta%20_4%7D%20-%20%7B%7B%5Crm%7BY%7D%7D_4%7D%7D%20%5Cend%7Barray%7D%7D%20%5Cright%5D%5Cleft%5B%20%7B%5Cbegin%7Barray%7D%7B*%7B20%7D%7Bc%7D%7D%20%7B%7B%7B%5Crm%7Bv%7D%7D_x%7D%7D%5C%5C%20%7B%7B%7B%5Crm%7Bv%7D%7D_y%7D%7D%5C%5C%20%7B%7B%7B%5Crm%7Bv%7D%7D_z%7D%7D%20%5Cend%7Barray%7D%7D%20%5Cright%5D%5C%3B)          （2.6）

即：   ![\Omega {\rm{ = }}\frac{1}{​{\rm{r}}}RV](https://private.codecogs.com/gif.latex?%5COmega%20%7B%5Crm%7B%20%3D%20%7D%7D%5Cfrac%7B1%7D%7B%7B%5Crm%7Br%7D%7D%7DRV)

根据第一图对θn的定义，可以得到θ1、θ3为 -45°,θ2、θ4为 +45°，因此R矩阵为：

                                             ![{\rm{R = }}\left[ {\begin{array}{*{20}{c}} 1&{ - 1}&{ - {X_1} - {​{\rm{Y}}_1}}\\ 1&1&{​{X_2} - {​{\rm{Y}}_2}}\\ 1&{ - 1}&{ - {X_3} - {​{\rm{Y}}_3}}\\ 1&1&{​{X_4} - {​{\rm{Y}}_4}} \end{array}} \right]](https://private.codecogs.com/gif.latex?%7B%5Crm%7BR%20%3D%20%7D%7D%5Cleft%5B%20%7B%5Cbegin%7Barray%7D%7B*%7B20%7D%7Bc%7D%7D%201%26%7B%20-%201%7D%26%7B%20-%20%7BX_1%7D%20-%20%7B%7B%5Crm%7BY%7D%7D_1%7D%7D%5C%5C%201%261%26%7B%7BX_2%7D%20-%20%7B%7B%5Crm%7BY%7D%7D_2%7D%7D%5C%5C%201%26%7B%20-%201%7D%26%7B%20-%20%7BX_3%7D%20-%20%7B%7B%5Crm%7BY%7D%7D_3%7D%7D%5C%5C%201%261%26%7B%7BX_4%7D%20-%20%7B%7B%5Crm%7BY%7D%7D_4%7D%7D%20%5Cend%7Barray%7D%7D%20%5Cright%5D)                                            （2.7）

通常我们设计的底盘是轴向对称的，也就是坐标系时原点到四个轮子的距离相等。所以每个轮横、纵坐标的绝对值各自相等，令K=abs(Xn)+abs(Yn)，根据每个轮坐标所在象限的符号，矩阵可化简为：

                                             ![{\rm{R = }}\left[ {\begin{array}{*{20}{c}} 1&{ - 1}&{ - {\rm{K}}}\\ 1&1&{ - {\rm{K}}}\\ 1&{ - 1}&{\rm{K}}\\ 1&1&{\rm{K}} \end{array}} \right]](https://private.codecogs.com/gif.latex?%7B%5Crm%7BR%20%3D%20%7D%7D%5Cleft%5B%20%7B%5Cbegin%7Barray%7D%7B*%7B20%7D%7Bc%7D%7D%201%26%7B%20-%201%7D%26%7B%20-%20%7B%5Crm%7BK%7D%7D%7D%5C%5C%201%261%26%7B%20-%20%7B%5Crm%7BK%7D%7D%7D%5C%5C%201%26%7B%20-%201%7D%26%7B%5Crm%7BK%7D%7D%5C%5C%201%261%26%7B%5Crm%7BK%7D%7D%20%5Cend%7Barray%7D%7D%20%5Cright%5D)                                                       （2.8）

另外，考虑î、û两个单位向量的线性组合，刚好构成XOY平面内的整个向量空间（当然实际上速度不可能无穷大的哈，但方向一定是全向的），这也就是麦轮能全向运动的原因。注意Bû始终是跟辊子轴垂直的，因为辊子一定是沿垂直其转轴的方向滚动，这就像Aî始终跟电机轴（平行于Y轴）垂直一样。

### **2.2 正解运动学模型**

正解运动学是指已知每个电机的转速，求出车体的速度。在数学形式上是找到一个3x4矩阵\[F\]使其满足：

                                               ![F\Omega r=V](https://latex.csdn.net/eq?F%5COmega%20r%3DV)                                                                         （2.9）

做法是通过最小二乘法来提供最优解的矩阵\[F\]，首先从逆向运动学方程开始，

                                            ![\Omega {\rm{ = }}\frac{1}{​{\rm{r}}}RV](https://private.codecogs.com/gif.latex?%5COmega%20%7B%5Crm%7B%20%3D%20%7D%7D%5Cfrac%7B1%7D%7B%7B%5Crm%7Br%7D%7D%7DRV)                                                                         （2.10）               

两边同时左乘\[R\]的转置得到：

                                             ![{R^{\rm{T}}}\Omega {\rm{ = }}\frac{1}{​{\rm{r}}}{R^{\rm{T}}}RV](https://private.codecogs.com/gif.latex?%7BR%5E%7B%5Crm%7BT%7D%7D%7D%5COmega%20%7B%5Crm%7B%20%3D%20%7D%7D%5Cfrac%7B1%7D%7B%7B%5Crm%7Br%7D%7D%7D%7BR%5E%7B%5Crm%7BT%7D%7D%7DRV)                                                               （2.11） 

两边同时左乘\[R\]’\[R\]的逆得到：

                                             ![{({R^{\rm{T}}}R)^{ - 1}}{R^{\rm{T}}}\Omega {\rm{ = }}\frac{1}{​{\rm{r}}}V](https://private.codecogs.com/gif.latex?%7B%28%7BR%5E%7B%5Crm%7BT%7D%7D%7DR%29%5E%7B%20-%201%7D%7D%7BR%5E%7B%5Crm%7BT%7D%7D%7D%5COmega%20%7B%5Crm%7B%20%3D%20%7D%7D%5Cfrac%7B1%7D%7B%7B%5Crm%7Br%7D%7D%7DV)                                                         （2.12）

右侧化简为\[V\]得到:

                                             ![{({R^{\rm{T}}}R)^{ - 1}}{R^{\rm{T}}}\Omega {\rm{r = }}V](https://private.codecogs.com/gif.latex?%7B%28%7BR%5E%7B%5Crm%7BT%7D%7D%7DR%29%5E%7B%20-%201%7D%7D%7BR%5E%7B%5Crm%7BT%7D%7D%7D%5COmega%20%7B%5Crm%7Br%20%3D%20%7D%7DV)                                                         （2.13）

由此得到\[F\]:

                                              ![{\rm{F}}\Omega {\rm{r = }}V\;,{\rm{F = }}{({R^{\rm{T}}}R)^{ - 1}}{R^{\rm{T}}}](https://latex.csdn.net/eq?%7B%5Crm%7BF%7D%7D%5COmega%20%7B%5Crm%7Br%20%3D%20%7D%7DV%5C%3B%2C%7B%5Crm%7BF%20%3D%20%7D%7D%7B%28%7BR%5E%7B%5Crm%7BT%7D%7D%7DR%29%5E%7B%20-%201%7D%7D%7BR%5E%7B%5Crm%7BT%7D%7D%7D)                                             （2.14）

使用逆向运动学中化简完的\[R\]矩阵，很容易求出\[F\]：这里我们借助MATLAB的符号函数求解 F矩阵，MATLAB代码如下所示

```
syms K
R=[1, -1, -K;
1, 1, -K;
1, -1, K;
1, 1, K;];
F=inv(R'*R)*R'
```

最终可以得到如下得一个F为下面的一个公式

![](https://i-blog.csdnimg.cn/blog_migrate/1951498f50d434a8e502ecd41d99f02c.png)

对应在程序中的[代码（github）](https://rupingcen.blog.csdn.net/article/details/102026459 "代码（github）")为：

![](https://i-blog.csdnimg.cn/blog_migrate/5a9f6c6ad5319054896232d6c4684594.png)

###  **2.3 逆解运动学模型**

小车的逆解模型主要用于已知整体速度求每个轮子的速度，比如遥控器对应的指令是小车整体的速度，我们接收到指令以后需要转换为每个轮子的目标转速作为速度的目标值

由2.1中可知

![](https://i-blog.csdnimg.cn/blog_migrate/683e509b9629e216a1b042b7faf741fd.png)

 由于车体的对称性质  K=abs(Xn)+abs(Yn)  因此有

![](https://i-blog.csdnimg.cn/blog_migrate/5125a09251ae456dc615ee5ef3bb42ca.png)

 对应的转换代码为（[github](https://rupingcen.blog.csdn.net/article/details/102026459 "github")）

![](https://i-blog.csdnimg.cn/blog_migrate/2b31c1022afc5e9f30981596414cb3e8.png)

**注意：**

      公式2.1中速度![V](https://private.codecogs.com/gif.latex?V)的单位是m/s，而转速![\Omega](https://private.codecogs.com/gif.latex?%5COmega)的单位为rad/s(弧度每秒)，大疆C620电机编码器反馈的速度为rpm（转没分），这其中需要转换一下（![\Omega](https://private.codecogs.com/gif.latex?%5COmega) \* 2\*![\pi](https://private.codecogs.com/gif.latex?%5Cpi) = rpm/60）

## 3 底盘软件框架

在上一章中，我们介绍了麦克纳姆轮底盘的基本框架，以及麦克纳姆轮运动学模型，这里我们通过大疆的遥控器以遥控的方式控制底盘实现前后左右运动。

底盘以STM32F1为核心,底盘可以分为以下几个功能模块：

1.  C620电调CAN驱动模块，通过CAN中断接收电机编码器反馈数据
2.  接收DBUS协议的遥控器数据
3.  设置速度PID闭环
4.  上传4个电机的转角、转速、温度到上位机

![](https://i-blog.csdnimg.cn/blog_migrate/b6fe1b1d0fa01a2ebadde1043da124e9.png)

具体而言首先使用串口1接收来至接收机的数据，然后由麦克纳姆轮运动学模型将X、Y、W三个方向上的速度转换为四个电机的目标转速（RPM）发送给PI控制作为目标速度，随后PI控制器接收四个轮子的目标转速实现速度闭环控制，并计算控制量，通过CAN总线传送到电机。最后由状态数据上传模块定时上传底盘上电机、IMU、超声波的数据到上位机上。

**接线说明：**

连线方面只需要把C620电调的CAN总线连上STM32、串口1接收引脚链接接收机（接收机与STM32之间需要外置一个反相器），串口2通过一条USB转串口的线连到上位机就可以了。

**注**：（大疆遥控器SW1需要拨到CL或者HL档位才有效，表示遥控器介入控制，OFF档表示撤销遥控器介入，由上位机进行控制）

代码就不贴出来了，感兴趣的小伙伴自行下载([底盘的代码](https://github.com/RuPingCen/mick_robot_chasiss "底盘的代码"))。到这里我们基本上可以实现用遥控器来遥控麦克纳姆轮底盘实现前、后、自旋三个方向的运动了。最后贴上一个效果图。

![](https://i-blog.csdnimg.cn/blog_migrate/40769788a0e40b3430b61dc197dee1ee.gif)

## 4 参考资料

\[1\]：大疆论坛，[【干货】麦轮正逆向运动学建模/再不懂就去打死线代老师吧【RoboMaster论坛-科技宅天堂】](https://bbs.robomaster.com/thread-3960-1-1.html "【干货】麦轮正逆向运动学建模/再不懂就去打死线代老师吧【RoboMaster论坛-科技宅天堂】")

\[2\]: 《Kinematic Analysis of Four-Wheel Mecanum Vehicle》

\[3\]: 代码地址：[GitHub - RuPingCen/mick\_robot\_chasiss: Mick is a homemade ROS chassis, which uses the Mecanum wheel as the wheel,](https://github.com/RuPingCen/mick_robot_chasiss "GitHub - RuPingCen/mick_robot_chasiss: Mick is a homemade ROS chassis, which uses the Mecanum wheel as the wheel,")

下一篇 ： [ROS底盘制作——麦克纳姆轮模型（下）](https://blog.csdn.net/crp997576280/article/details/102487407 "ROS底盘制作——麦克纳姆轮模型（下）")

对于运行有问题的小伙伴欢迎邮件或留言讨论cenruping@vip.qq.com