#### 2.6自启动脚本问题

首先我们延续之前的教程，我们将启动所有ROS节点的命令放在一个自启动脚本中，让Ubuntu开机自动运行，在Ubuntu中有两种方式可以实现自启动脚本的功能。

##### 第1种 ubuntu自带的 startup Application

首先新建一个mytest.sh文件 然后填入以下内容 \[6\]，这里是启动了一个turtlebot的节点和一个小海龟的例子

```cpp
#! /bin/bash
### BEGIN INIT INFO
# Provides:          xxxx.com
# Required-Start:    $local_fs $network
# Required-Stop:     $local_fs
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: mylaunch service
# Description:       mylaunch service test
### END INIT INFO

gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;roslaunch turtlebot_bringup minimal.launch" #新建终端启动节点
sleep 8 #等待8秒
gnome-terminal -x bash -c "source /opt/ros/kinetic/setup.bash;rosrun turtlesim turtlesim_node" &  
wait
exit 0
```

其中gnome-terminal -x bash -c表示新建bash终端并执行c后面的语句,命令通过’;;’,隔开,  
!注意,如果是ubuntu 18.04的系统 \[6\], **gnome-terminal -x bash -c需要改写成gnome-terminal – bash -c**

接下来，对 mytest.sh 文件修改权限

```bash
sudo chmod 777 mytest.sh
```

这里最好是先测试一下你自己写的.sh文件是否可以运行

```bash
./mytest.sh
```

在ubuntu 1604中搜索 “startup Application” 会找到配置软件  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/c91a2bc05660e95d41f1b336c566244f.png#pic_center)  
选定我们新建的mytest.sh文件  
![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/7624e37304493ea0bd8a12fa9ecde336.png#pic_center)  
这里需要注意，我们需要把当前的账户设置为自动登录

![在这里插入图片描述](https://i-blog.csdnimg.cn/blog_migrate/0e15cb53d47c0f244407dfd8e51e3859.png#pic_center)  
这样重启以后会自动进入桌面然后ros节点，注（我在测试的过程中发现，这种方法必须要设置账户自动登录，应该是进入桌面以后才能运行我们设置的启动脚本）

##### 第2种 rc.local 脚本启动

第2种是通过Ubuntu 1604 中的rc.local 这个脚本文件（/etc/rc.local），这里我们需要预先将启动的代码编写到一个xxx.sh 文件里面（当然，如果命令少的话你也可以将所有的命令直接卸载rc.local 中），然后打开rc.local文件在 “exit 0 前面添加自启动的命令” 如：

```bash
gnome-terminal --window -x bash -c "sh /home/crp/mytest.sh;exec bash"
```

这里还有另外一种命令格式：

```bash
gnome-terminal --window -x bash -c "sh /home/crp/mytest.sh;exec bash"
echo  123456| sudo -S 
gnome-terminal -t "terminal-name" -X bash -c "sh ./xxx.sh"; exec bash;
```

这段命令是说，新建一个terminal终端，然后运行在 /home/crp/xxx.sh 目录下的 xxx.sh 文件， 同时切换为超级用户(密码为123456)，之后再新建一个终端运行另外一个xxx.sh文件。

上述方式可能会失效，\[7\] 使用gnome-terminal --window -x bash -c 这种弹窗命令是不生效的。

##### 第3种 安装脚本服务启动

第3种方式是将启动脚本安装为Ubuntu的启动服务：  
1> 将启动脚本 “my\_autorun.sh” 复制到 /etc/init.d/ 目录下  
2> 接下来修改这个脚本的权限

```
sudo chmod 755 /etc/init.d/mytest.sh
```

3> 安装脚本服务

```
cd /etc/init.d
sudo update-rc.d mytest.sh defaults 95   #其中95是启动顺序
```

4> 卸载脚本服务(安装时候不需要操作)

```
 cd /etc/init.d
 sudo update-rc.d -f mytest.sh remove   #其中95是启动顺序
```

接下来我们新建一下名为“car\_launch.sh”的文件，并添加以下内容：

### 3 小结

通过前面三篇的博客，梳理了自主导航小车的硬件框架，建图的算法框架以及导航的算法框架。所有的代码我都放在了这个[github](https://github.com/RuPingCen/P3AT_Auto_Navigation)上。

在硬件篇中，由于我们使用的是标准平台，不涉及自己制作小车底盘，因此没有提及到底盘制作的步骤，后续将搭建一个自制的ROS底盘，一般来讲ROS底盘需要接收线速度和角度指令，底盘由单片机与ROS节点通讯，单片机负责电机的闭环控制，以及底盘的运动学模型转换。此外底盘上还需要有超声波和电源管理系统等额外传感模块。

在建图的过程中我们只是用了gmapping建图方式，gmapping只适用于2D的激光雷达，对于3D激光和RGB-D相机没法建图，因此后续计划学习google的cartography已经使用ORB-SLAM建立3D点云地图。

在运动规划过程中，我们只是浅显的理解了move\_base的运行参数，由于其参数比较多，因此计划在后续过程中更加深入的理解move\_base的运行参数

### 参考资料

【1】https://blog.csdn.net/huaweijian0324/article/details/81142610  
【2】 https://www.ncnynl.com/category/ros-car/  
【3】https://www.ncnynl.com/archives/201703/1415.html  
【4】https://blog.csdn.net/Nksjc/article/details/77359952  
【5】https://blog.csdn.net/x\_r\_su/article/details/52927564  
【6】https://blog.csdn.net/baidu\_34319491/article/details/106456571  
【7】https://blog.csdn.net/Andrwin/article/details/100883593

上一篇 ： [自主导航小车实践（三）](https://blog.csdn.net/crp997576280/article/details/99702835)

对于运行有问题的小伙伴欢迎邮件或留言讨论cenruping@vip.qq.com