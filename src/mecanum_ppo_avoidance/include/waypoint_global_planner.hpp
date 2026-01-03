#ifndef WAYPOINT_GLOBAL_PLANNER_H
#define WAYPOINT_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

namespace waypoint_global_planner
{
    class WayPointGlobalPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        WayPointGlobalPlanner();
        ~WayPointGlobalPlanner();
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) override;
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan) override;
        void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void externalPathCallback(const nav_msgs::PathConstPtr &plan);
        void createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped> &path);

    private:
        bool initialized_;             /* 控制器是否被初始化 */
        ros::Subscriber waypoint_sub_; /* 订阅路径点的订阅者 */
    };

}

#endif // !WAYPOINT_GLOBAL_PLANNER_H