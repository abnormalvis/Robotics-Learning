[DynamicX Docs](http://dynamicx.top/index.html)

–作者：冠霖–

## gpio\_controller[](#gpio-controller "Permalink to this heading")

## 1\. init[](#init "Permalink to this heading")

1.  获取参数文件中的gpios，也就是gpio名字
    
2.  初始化一个实时发布者
    
3.  一个for循环，遍历每一个gpio
    
4.  一个订阅者，接收指令
    

## 2\. update[](#update "Permalink to this heading")

2.  遍历每一个gpio，[把每一个gpio的value写到msg\_](#id5).gpio\_state\[\]
    
3.  写时间戳
    
4.  解锁并发布消息
    

## 3\. setGpioCmd[](#setgpiocmd "Permalink to this heading")

1.  这是/command这个话题订阅者的回调函数
    
2.  遍历每一个gpio\_command\_handle，在msg中如果找到了对应的gpio name的话，就setcommand
    

## orientation\_controller(冠霖)[](#orientation-controller "Permalink to this heading")

**before reading**

```
bool Controller::getTransform(const ros::Time& time, geometry_msgs::TransformStamped& source2target, const double x,
                              const double y, const double z, const double w)
{
  source2target.header.frame_id = frame_source_;
  source2target.child_frame_id = frame_target_;
  source2target.transform.rotation.w = 1.0;
  tf2::Transform source2odom, odom2fixed, fixed2target;
  try   //给这三个Transform赋值
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_.lookupTransform(frame_source_, "odom", time);
    tf2::fromMsg(tf_msg.transform, source2odom);
    tf_msg = robot_state_.lookupTransform("odom", imu_sensor_.getFrameId(), time);
    tf2::fromMsg(tf_msg.transform, odom2fixed);
    tf_msg = robot_state_.lookupTransform(imu_sensor_.getFrameId(), frame_target_, time);
    tf2::fromMsg(tf_msg.transform, fixed2target);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
  tf2::Quaternion odom2fixed_quat;
  odom2fixed_quat.setValue(x, y, z, w); //用imu的orientation设置odom2fixed的rotation，也就是用imu进行修正
  odom2fixed.setRotation(odom2fixed_quat);
  source2target.transform = tf2::toMsg(source2odom * odom2fixed * fixed2target);
  return true;
}

/*
假设：初始odom base imu三个坐标系重合
令云台旋转90度，此时imu知道它此时的yaw=90度（记为yaw_real），而在tf中认为imu的yaw为91度（记为yaw）

计算source2odom * odom2fixed * fixed2target；odom2fixed是imu在odom的真实姿态（90度），乘fixed2target后（往回转91度），就得到了修正的base与odom差1度

计算source2odom * odom2fixed * fixed2target得到一个odom2base的转换，他们之间只有一个yaw的偏移，偏移量应该就是yaw与yaw_real之间的角度差，注意base2pitch是不变的，但是odom2pitch是改变了的，此时他跟odom2fixed是一样的，也就是说修正过后，在世界坐标系下，我的云台姿态跟imu姿态是一致的
```