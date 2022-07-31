#ifndef MULTI_FLOOR_NAVIGATION_H
#define MULTI_FLOOR_NAVIGATION_H

#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"

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
        bool received_amcl_pose, goal_sent, goal_active;
        double loop_rate, max_linear_error, max_angular_error;
        ros::Publisher initial_pose_pub, goal_pub, cmd_vel_pub;
        ros::Subscriber amcl_pose_sub, odom_sub, move_base_status_sub;
        geometry_msgs::PoseWithCovarianceStamped curr_pose;
        nav_msgs::Odometry curr_odom, first_odom;
        actionlib_msgs::GoalStatusArray move_base_status_msg;


        tf2::Quaternion convertYawtoQuartenion(double yaw);
        void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void movebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
        void init_pose(double x, double y, double z, double yaw);
        bool check_robot_pose(double x, double y, double yaw);
        void send_simple_goal(double x, double y, double z, double yaw);
        void send_cmd_vel(double x_vel, double theta_vel);


    public:
        MultiFloorNav();
        void initialize (ros::NodeHandle& n);
        double getLoopRate();
        void execute();
        double test;
};

#endif