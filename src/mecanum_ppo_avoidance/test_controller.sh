#!/bin/bash
# 麦轮底盘控制器测试脚本

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== 麦轮底盘控制器测试脚本 ===${NC}"
echo ""

# 1. Source workspace
echo -e "${YELLOW}[1/4] Sourcing workspace...${NC}"
source /home/idris/As-my-see/robotics-learning/devel/setup.bash
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Workspace sourced successfully${NC}"
else
    echo -e "${RED}✗ Failed to source workspace${NC}"
    exit 1
fi
echo ""

# 2. 启动Gazebo和RViz
echo -e "${YELLOW}[2/4] 启动Gazebo和RViz...${NC}"
echo "正在后台启动display_with_tf.launch..."
roslaunch mecanum_ppo_avoidance display_with_tf.launch > /home/idris/As-my-see/robotics-learning/src/mecanum_ppo_avoidance/logs/display_with_tf.log 2>&1 &
DISPLAY_PID=$!
echo -e "${GREEN}✓ Display launched (PID: $DISPLAY_PID)${NC}"
echo "等待Gazebo启动完成..."
sleep 8
echo ""

# 3. 加载控制器
echo -e "${YELLOW}[3/4] 加载ros_control控制器...${NC}"
roslaunch mecanum_ppo_avoidance load_controller.launch > /home/idris/As-my-see/robotics-learning/src/mecanum_ppo_avoidance/logs/load_controller.log 2>&1 &
CONTROLLER_PID=$!
echo -e "${GREEN}✓ Controller loaded (PID: $CONTROLLER_PID)${NC}"
echo "等待控制器加载完成..."
sleep 5
echo ""

# 4. 显示测试信息
echo -e "${YELLOW}[4/4] 测试信息${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${GREEN}✓ 所有组件已启动！${NC}"
echo ""
echo "📝 日志文件位置:"
echo "   - Display log: logs/display_with_tf.log"
echo "   - Controller log: logs/load_controller.log"
echo ""
echo "🎮 测试命令:"
echo "   # 查看话题"
echo "   rostopic list"
echo ""
echo "   # 发送速度命令 (前进)"
echo "   rostopic pub /cmd_vel geometry_msgs/Twist \"linear:"
echo "     x: 0.5"
echo "     y: 0.0"
echo "     z: 0.0"
echo "   angular:"
echo "     x: 0.0"
echo "     y: 0.0"
echo "     z: 0.0\" -r 10"
echo ""
echo "   # 查看里程计"
echo "   rostopic echo /odom"
echo ""
echo "   # 查看关节状态"
echo "   rostopic echo /joint_states"
echo ""
echo "   # 使用键盘遥控"
echo "   rosrun teleop_twist_keyboard teleop_twist_keyboard.py"
echo ""
echo "🛑 停止测试:"
echo "   Press Ctrl+C then run: killall -9 gzserver gzclient rosmaster roscore"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# 保持脚本运行
echo "按 Ctrl+C 停止所有节点..."
trap "echo ''; echo 'Stopping all nodes...'; kill $DISPLAY_PID $CONTROLLER_PID; killall -9 gzserver gzclient 2>/dev/null; exit" INT
wait
