#ifndef MULTI_FLOOR_NAVIGATION_H
#define MULTI_FLOOR_NAVIGATION_H

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MultiFloorNav{
    private:
        enum State{
            INIT_POSE, NAV_TO_WP_1, DONE
        };
        State nav_state;
        double loop_rate;
        ros::Publisher initial_pose_pub, goal_pub;
        ros::Subscriber amcl_pose_sub;
        geometry_msgs::PoseWithCovarianceStamped curr_pose;

        tf2::Quaternion convertYawtoQuartenion(double yaw);
        void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void init_pose(double x, double y, double z, double yaw);
        void send_simple_goal(double x, double y, double z, double yaw);

    public:
        MultiFloorNav();
        void initialize (ros::NodeHandle& n);
        double getLoopRate();
        void execute();
        double test;
};

#endif