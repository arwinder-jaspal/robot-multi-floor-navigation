#include <multi_floor_nav/multi_floor_navigation.h>
// #include "multi_floor_navigation.h"

using namespace std;

MultiFloorNav::MultiFloorNav(){
    nav_state = MultiFloorNav::State::INIT_POSE;
    received_amcl_pose = false;
    goal_active = false;
    goal_sent = false;
    loop_rate = 1.0;
    max_angular_error = 0.0;
    max_linear_error = 0.0;
    curr_odom.pose.pose.orientation = first_odom.pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0); 
}

void MultiFloorNav::initialize(ros::NodeHandle& n){
    ros::NodeHandle np("~");

    ros::param::param<double>("~loop_rate", loop_rate, 5.0);
    ros::param::param<double>("~max_angular_error", max_angular_error, 0.05);
    ros::param::param<double>("~max_linear_error", max_linear_error, 0.5);

    initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    elevator_pub = n.advertise<std_msgs::String>("elevator", 1, true);
    
    amcl_pose_sub = n.subscribe("amcl_pose", 1, &MultiFloorNav::amclPoseCallback, this);
    odom_sub = n.subscribe("odometry/filtered", 1, &MultiFloorNav::odomCallback, this);
    move_base_status_sub = n.subscribe("move_base/status", 1, &MultiFloorNav::movebaseStatusCallback, this);
}

void MultiFloorNav::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    received_amcl_pose = true;
    curr_pose.pose = msg->pose;
}

void MultiFloorNav::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    curr_odom = *msg;
}

void MultiFloorNav::movebaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
    move_base_status_msg = *msg;
}
tf2::Quaternion MultiFloorNav::convertYawtoQuartenion(double yaw){
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    quat.normalize();
    return quat;
}

void MultiFloorNav::init_pose(double x, double y, double z, double yaw){
    geometry_msgs::PoseWithCovarianceStamped init_pose;

    init_pose.header.frame_id = "map";
    init_pose.header.stamp = ros::Time::now();

    init_pose.pose.pose.position.x = x;
    init_pose.pose.pose.position.y = y;
    init_pose.pose.pose.position.z = z;

    tf2::Quaternion quat;
    quat = convertYawtoQuartenion(yaw);

    init_pose.pose.pose.orientation.x= quat[0];
    init_pose.pose.pose.orientation.y= quat[1];
    init_pose.pose.pose.orientation.z= quat[2];
    init_pose.pose.pose.orientation.w= quat[3];

    ROS_INFO("[Multi Floor Nav] Initializing Robot at x: %.1f, y: %.1f, z: %.1f, yaw: %.1f", x, y, z, yaw);
    initial_pose_pub.publish(init_pose);
}
bool MultiFloorNav::check_robot_pose(double ground_truth_x, double ground_truth_y, double ground_truth_yaw){
    while (!received_amcl_pose);
    
    double amcl_x = curr_pose.pose.pose.position.x;
    double amcl_y = curr_pose.pose.pose.position.y;
    double diff_x = fabs(ground_truth_x - amcl_x);
    double diff_y = fabs(ground_truth_y - amcl_y);
    double linear_error = sqrt(pow(diff_x,2) + pow(diff_y,2)); //Find linear error
    double amcl_yaw = tf::getYaw(curr_pose.pose.pose.orientation);
    double angular_error = fabs(angles::normalize_angle(ground_truth_yaw-amcl_yaw));
    ROS_INFO(
        "[Multi Floor Nav] Current Robot Pose, x: %.2f, y: %.2f, yaw: %.2f",amcl_x, amcl_y, amcl_yaw);
    ROS_INFO("[Multi Floor Nav] Linear Error: %.2f, Angular Error: %.2f",linear_error, angular_error);
    if((linear_error < max_linear_error))
        return true;
    else 
        return false;
}
void MultiFloorNav::send_simple_goal(double x, double y, double z, double yaw){
    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    
    tf2::Quaternion quat;
    quat = convertYawtoQuartenion(yaw);
    
    goal.pose.orientation.x = quat[0];
    goal.pose.orientation.y = quat[1];
    goal.pose.orientation.z = quat[2];
    goal.pose.orientation.w = quat[3];

    ROS_INFO("[Multi Floor Nav] Sending Robot to x: %.1f, y: %.1f, z: %.1f, yaw: %.1f", x, y, z, yaw);
    goal_pub.publish(goal);
}

void MultiFloorNav::send_cmd_vel(double x_vel=0.0, double theta_vel=0.0){
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = x_vel;
    cmd_vel.angular.z = theta_vel;
    cmd_vel.linear.y = cmd_vel.linear.z = cmd_vel.angular.x = cmd_vel.angular.y = 0.0;

    cmd_vel_pub.publish(cmd_vel);
}

double MultiFloorNav::getYawOffset(geometry_msgs::Quaternion A, geometry_msgs::Quaternion B) {
    tf::Matrix3x3 a_mat(tf::Quaternion(A.x, A.y, A.z, A.w));
    double a_yaw, a_pitch, a_roll;
    a_mat.getEulerYPR(a_yaw, a_pitch, a_roll);

    tf::Matrix3x3 b_mat(tf::Quaternion(B.x, B.y, B.z, B.w));
    double b_yaw, b_pitch, b_roll;
    b_mat.getEulerYPR(b_yaw, b_pitch, b_roll);
    return angles::normalize_angle(a_yaw - b_yaw);
}

void MultiFloorNav::align_yaw(double yaw_offset, double angular_vel){
    if(yaw_offset>0.0)
        send_cmd_vel(0.0,-angular_vel);
    else
        send_cmd_vel(0.0,angular_vel);
}

void MultiFloorNav::request_lift(string floor){
    std_msgs::String floor_id;

    floor_id.data = floor;

    elevator_pub.publish(floor_id);
}

double MultiFloorNav::dist(geometry_msgs::Point A, geometry_msgs::Point B) {
  return length(B.x - A.x, B.y - A.y, A.z - B.z);
}

double MultiFloorNav::length(double diff_x, double diff_y, double diff_z) {
  return sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
}
double MultiFloorNav::getPositionOffset(geometry_msgs::Point A, geometry_msgs::Point B) {
  return dist(A, B);
}   

bool MultiFloorNav::reached_distance(double distance){
  double distance_travelled = getPositionOffset(
    curr_odom.pose.pose.position, first_odom.pose.pose.position);
  if(distance_travelled >= distance){
    return true;
  }
  return false;
}

void MultiFloorNav::execute(){
    ROS_INFO("[Multi Floor Nav] State: %d", nav_state);
    switch(nav_state){
        case MultiFloorNav::State::INIT_POSE:
            init_pose(4.0,-5.0, 0.5, 0.0);
            nav_state = MultiFloorNav::State::CHECK_INITPOSE;
            ros::Duration(1.0).sleep();
            break;
        case MultiFloorNav::State::CHECK_INITPOSE:
            if(check_robot_pose(4.0, -5.0, 0.0)){
                ROS_INFO("[Multi Floor Nav] Robot at correct pose");
                nav_state = MultiFloorNav::State::NAV_TO_WP_1;
                ros::Duration(1.0).sleep();
            }
            else{
                ROS_INFO("[Multi Floor Nav] Setting initial pose failed. Will Retry");
                nav_state = MultiFloorNav::State::INIT_POSE;
            }
            break;
        case MultiFloorNav::State::NAV_TO_WP_1:
            ROS_INFO_ONCE("[Multi Floor Nav] Will send goal to x: %.f, y: %.f, z: %.f", 3.0, -0.5, 0.5); 
            if(!goal_sent){
                send_simple_goal(3.0, -0.5, 0.5, M_PI);
                goal_sent = true;
            }
            if(!move_base_status_msg.status_list.empty()){
                if(move_base_status_msg.status_list[0].status == actionlib_msgs::GoalStatus::ACTIVE){
                goal_active = true;
                }
                else if(goal_active && move_base_status_msg.status_list[0].status == actionlib_msgs::GoalStatus::SUCCEEDED){
                nav_state = MultiFloorNav::State::ALIGN_ROBOT_LIFT_LEVEL_0;
                goal_sent = goal_active = false;
                first_odom = curr_odom;
                send_cmd_vel(); //stops the amr in case of still oscillating (theorectically shouldn't)
                }
            }
            break;
        case MultiFloorNav::State::ALIGN_ROBOT_LIFT_LEVEL_0:
            double yaw_diff;
            current_yaw = curr_odom.pose.pose.orientation;
            desired_yaw = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, M_PI);
            yaw_diff = getYawOffset(current_yaw, desired_yaw);
            if(abs(yaw_diff) > max_angular_error){
                align_yaw(yaw_diff, 0.2);
            }
            else{
                send_cmd_vel();
                nav_state = MultiFloorNav::State::SEND_LIFT_0;
            }
            break;
        case MultiFloorNav::State::SEND_LIFT_0:
            request_lift("0");
            nav_state= MultiFloorNav::State::ENTER_LIFT_LEVEL_0;
            break;

        case MultiFloorNav::State::ENTER_LIFT_LEVEL_0:
            // request_lift("0");
            send_cmd_vel(0.25); 
            if(reached_distance(3.0)){
                send_cmd_vel();
                nav_state= MultiFloorNav::State::DONE;
                first_odom = curr_odom;
                ros::Duration(5).sleep(); // wait 5 sec for the lift to close
            }      
            break;
        case MultiFloorNav::State::DONE:
            ROS_INFO_ONCE("Robot arrived at Goal");
            break;
    }

}

double MultiFloorNav::getLoopRate(){
    return loop_rate;
}

int main(int argc, char** argv) {   
    ros::init(argc, argv, "multi_floor_navigation_node");
    ros::NodeHandle n;

    MultiFloorNav multi_floor_nav;
    multi_floor_nav.initialize(n);
    double rate = multi_floor_nav.getLoopRate();
    if(rate <= 0.0){
        rate = 1.0;
    }
    ros::Rate loop_rate(rate);

while (ros::ok()) {
ros::spinOnce();
multi_floor_nav.execute();
loop_rate.sleep();
}

ros::spin();

return (0);
}

