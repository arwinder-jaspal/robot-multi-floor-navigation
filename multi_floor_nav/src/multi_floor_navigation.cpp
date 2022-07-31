#include <multi_floor_nav/multi_floor_navigation.h>
// #include "multi_floor_navigation.h"

using namespace std;

MultiFloorNav::MultiFloorNav(){
    nav_state = MultiFloorNav::State::INIT_POSE;
    loop_rate = 1.0;
}

void MultiFloorNav::initialize(ros::NodeHandle& n){
    ros::NodeHandle np("~");

    ros::param::param<double>("~loop_rate", loop_rate, 1.0);

    initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    amcl_pose_sub = n.subscribe("amcl_pose", 1, &MultiFloorNav::amclPoseCallback, this);
}

void MultiFloorNav::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    curr_pose.pose = msg->pose;
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

void MultiFloorNav::execute(){
    switch(nav_state){
        case MultiFloorNav::State::INIT_POSE:
            init_pose(4.0,-5.0, 0.5, 0.0);
            nav_state = MultiFloorNav::State::DONE;
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

