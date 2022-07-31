#ifndef CHANGE_MAP_H
#define CHANGE_MAP_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <multi_floor_nav/IntTrigger.h>

class ChangeMap{
    private:
        ros::Publisher map_pub;
        ros::Subscriber map_level_0_sub, map_level_1_sub, desired_level_sub;
        ros::ServiceServer change_map_server;

        nav_msgs::OccupancyGrid map_level_0, map_level_1, map;
        double loop_rate;
        void mapLevelZeroCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void mapLevelOneCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        bool changeMapCallback(multi_floor_nav::IntTrigger::Request &req, 
                               multi_floor_nav::IntTrigger::Response &res);

    public:
        ChangeMap();
        void initialize (ros::NodeHandle& n);
        double getLoopRate();

};
#endif