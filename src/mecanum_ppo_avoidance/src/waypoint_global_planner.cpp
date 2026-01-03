#include <waypoint_global_planner.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>

namespace waypoint_global_planner
{
    WayPointGlobalPlanner::WayPointGlobalPlanner() : initialized_(false) {}

    WayPointGlobalPlanner::~WayPointGlobalPlanner() {}

    void WayPointGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (initialized_)
            return;
        ros::NodeHandle nh(name);
        (void)costmap_ros;
        initialized_ = true;
    }

    bool WayPointGlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                                         std::vector<geometry_msgs::PoseStamped> &plan)
    {
        (void)start;
        (void)goal;
        plan.clear();
        return true;
    }

}

PLUGINLIB_EXPORT_CLASS(waypoint_global_planner::WayPointGlobalPlanner, nav_core::BaseGlobalPlanner)