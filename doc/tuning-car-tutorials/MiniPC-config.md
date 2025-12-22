## nuc基本配置[](#nuc "Permalink to this heading")

## 安装Ubuntu服务器版[](#ubuntu "Permalink to this heading")

插入装有ubuntu服务器版的u盘，进入u盘启动，正常安装即可。给/swap分配16g，/efi分配512m，剩下的硬盘空间全部挂载到/目录下。设备名看这台nuc给哪个机器人用，用户名dynamicx，密码dynamicx。安装完成后按照提示移除u盘，重启。

## 安装ssh，并通过ssh用你的电脑操控nuc[](#ssh-sshnuc "Permalink to this heading")

1、将自己的电脑连接上wifi，然后用一根网线将nuc和你的电脑连接。这时你的电脑左上方会出现有线连接的图标。进入有线连接，点击设置，点击IPV4,设置“与其他计算机共享网络”（类似的意思）。这样nuc就能上网了。可用\`ping baidu.com\`来检查是否已联网。

2、安装ssh服务器端

```
sudo apt install openssh-server
```

**如需要安装依赖，按提示安装**

3、在nuc上使用\`ip a\`查看nuc的ip地址。在你的电脑上输入指令\`ssh dynamicx@“nuc的ip地址”[\`](#id1)，这样你就能在你的电脑上操控nuc了。

## 换源[](#id3 "Permalink to this heading")

网上搜索国内的源，例如清华源，阿里源，换源。网上的清华源有的能用，有的是坏的，如果装清华源锅了就换一个。

## 安装easywifi[](#easywifi "Permalink to this heading")

1、在github上搜索easywifi,第一个就是。将源代码clone下来，注意要用http。

2、安装easywifi依赖：

```
sudo apt-get install network-manager-config-connectivity-ubuntu
```

3、进入easywifi文件夹，输入

```
sudo python3 easywifi.py
```

4、成功运行easywifi，运行 [\*](#id4)1\*<!–Scan for networks–> 搜索wifi，然后运行 [\*](#id6)5\*<!–Setup new network–> 输入wifi名称和密码，让nuc连上wifi。

5、和nuc连上同一个wifi，继续用ssh操控nuc。

## 安装ros[](#ros "Permalink to this heading")

安装建议直接搜索鱼香ROS,在自己的电脑上操作.

### 安装catkin tools[](#catkin-tools "Permalink to this heading")

catkin tools官方文档：[https://catkin-tools.readthedocs.io/en/latest/](https://catkin-tools.readthedocs.io/en/latest/) 如果你使用catkin build时需要你安装osrf-pycommon>0.1.1这个依赖，请输入以下指令：

```
sudo apt-get install python3-pip
pip3 install osrf-pycommon
```

### rosdep[](#rosdep "Permalink to this heading")

rosdep update 失败的参考解决方法：[https://github.com/SparkChen927/rosdep](https://github.com/SparkChen927/rosdep)

```
git clone https://gitclone.com/github.com/SparkChen927/rosdep.git
```

## 为机器人安装软件包[](#id8 "Permalink to this heading")

我们团队为 **rm\_control** 和 **rm\_controllers** 搭建了软件源，请根据 [这个网站](https://rm-control-docs.netlify.app/quick_start/rm_source) 给nuc添加软件源并把相应的软件包拉下来。

## 免密登陆设置[](#id10 "Permalink to this heading")

当我们远程连接NUC的时候，需要输入密码，但是这样的话可能会比较麻烦，因此最好我们配置免密登陆，可以省去很多麻烦

命令行操作：

```
ssh-copy-id dynamicx@host
```

回车之后还需要输入一次密码，输入完之后就可以了，以后每次都可以免密登陆

## 优化[](#id11 "Permalink to this heading")

1、你会发现开机很慢，这是一个系统服务导致的，可以设置将其跳过。

```
$ sudo vim /etc/netplan/01-netcfg.yaml`
```

注：这个文件可能不叫这个名字，可能需要转到/etc/netplan这个目录下看看。

在网卡的下一级目录中增加

```
optional: true
```

修改完后生效设置

```
$ sudo netplan apply
```

2、阻止nuc休眠

nuc长时间不用会休眠，会给工作带来一定麻烦。因此需要设置阻止nuc休眠。输入以下指令：

```
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

## 换内核[](#id12 "Permalink to this heading")

1、使用搜索引擎搜索xanmod，通常搜索结果第一个就是，打开 [此网站](https://xanmod.org/) 。

2、我们需要更换一个实时性更强的内核，这样的内核通常名字里会带有“rt”（realtime）。在这个网站往下拉会看到“Install via Terminal”(通过命令行安装)。根据提示安装自己想要的内核（目前队内使用的是实时性内核为: **Linux version 6.1.38-rt12-x64v3-xanmod1** ）。

3、使用指令 `sudo dpkg --get-selections | grep linux-image` 来查看你想要安装的内核是否安装成功。

4、重启，按F2进入BIOS模式。在boot->Boot Priority勾选Fast boot。Power选项里勾选Max Performance Enabled,Dynamic Power Technology设为最长的那个，Power->Secondary Power Settings将After Power Failure设为Power on。cooling选项里将Fan Control Mode设置为Fixed，Fixed Duty Cycle设为100。 **关闭安全启动** 然后退出BIOS，正常启动。

5、测试新内核的实时性和can总线传输速率

P.S: 1、如果进不了BIOS可尝试长按开机键直至指示灯变成橙色。

2、如果能进入linux系统，那么可以执行 `sudo systemctl reboot --firmware-setup` ，重启直接不进入系统，而进入bios。

## 自启脚本[](#id14 "Permalink to this heading")

**部署自启**

1.  将rm\_bringup传到nuc上，编译
    
2.  输入命令行
    

```
:~$ roscd rm_bringup/scripts/auto_start
:~$ ./creat_specific_service.sh
```

1.  修改rm\_start.sh
    

+   将ROBOT\_TYPE改为对应车的类型
    

1.  重启nuc
    

**rosbag**

传入bringup，前往auto\_start执行

./create\_special\_service.sh control\_rosbag\_record

./create\_special\_service.sh memeory\_minitor //这个会检测磁盘空间，删除多的rosbag