#ifndef MULTI_FLOOR_NAVIGATION_H
#define MULTI_FLOOR_NAVIGATION_H

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <math.h>


class MultiFloorNav{
    private:
        enum State{
            INIT_POSE,
            CHECK_INITPOSE, 
            NAV_TO_WP_1, 
            DONE
        };
        State nav_state;
        bool received_amcl_pose;
        double loop_rate, max_linear_error, max_angular_error;
        ros::Publisher initial_pose_pub, goal_pub;
        ros::Subscriber amcl_pose_sub;
        geometry_msgs::PoseWithCovarianceStamped curr_pose;

        tf2::Quaternion convertYawtoQuartenion(double yaw);
        void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void init_pose(double x, double y, double z, double yaw);
        void send_simple_goal(double x, double y, double z, double yaw);
        bool check_robot_pose(double x, double y, double yaw);

    public:
        MultiFloorNav();
        void initialize (ros::NodeHandle& n);
        double getLoopRate();
        void execute();
        double test;
};

#endif