#ifndef MULTI_FLOOR_NAVIGATION_H
#define MULTI_FLOOR_NAVIGATION_H

#include <ros/ros.h>


#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Empty.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <math.h>
#include <multi_floor_nav/IntTrigger.h>


class MultiFloorNav{
    private:
        enum State{
            LOAD_MAP,
            INIT_POSE,
            CHECK_INITPOSE, 
            NAV_TO_GOAL,
            ALIGN_ROBOT_LIFT_LEVEL_0,
            SEND_LIFT_0,
            ENTER_LIFT_LEVEL_0, 
            SEND_LIFT_1,
            EXIT_LIFT_LEVEL_1,
            DONE
        };
        State nav_state;
        int desired_map_level;
        bool received_amcl_pose, goal_sent, goal_active, to_start;
        double loop_rate, max_linear_error, max_angular_error;
        ros::Publisher initial_pose_pub, goal_pub, cmd_vel_pub, elevator_pub;
        ros::Subscriber amcl_pose_sub, odom_sub, move_base_status_sub, start_sub;
        ros::ServiceClient change_map_client;
        geometry_msgs::PoseWithCovarianceStamped curr_pose;
        nav_msgs::Odometry curr_odom, first_odom;
        actionlib_msgs::GoalStatusArray move_base_status_msg;
        geometry_msgs::Quaternion current_yaw, desired_yaw;
        geometry_msgs::Pose2D desired_init_pose, desired_goal_pose;
        multi_floor_nav::IntTrigger srv;

        void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void movebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
        void startCallback(const std_msgs::Empty::ConstPtr& msg);

        tf2::Quaternion convertYawtoQuartenion(double yaw);
        void set_init_pose(geometry_msgs::Pose2D pose);
        bool check_robot_pose(geometry_msgs::Pose2D pose);
        void send_simple_goal(geometry_msgs::Pose2D goal_pose);
        void send_cmd_vel(double x_vel, double theta_vel);
        double getYawOffset(geometry_msgs::Quaternion A, geometry_msgs::Quaternion B);
        void align_yaw(double yaw_offset, double angular_vel);
        void request_lift(std::string floor);
        double dist(geometry_msgs::Point A, geometry_msgs::Point B);
        double length(double x, double y, double z = 0);
        double getPositionOffset(geometry_msgs::Point A, geometry_msgs::Point B);
        bool reached_distance(double distance);
        void set_desired_level(int level_id);

    public:
        MultiFloorNav();
        void initialize (ros::NodeHandle& n);
        double getLoopRate();
        void execute();
        double test;
};

#endif