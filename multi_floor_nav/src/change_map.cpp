#include <multi_floor_nav/change_map.h>

using namespace std;

ChangeMap::ChangeMap(){
    loop_rate = 1.0;
}

void ChangeMap::initialize(ros::NodeHandle& n){
    ros::NodeHandle np("~");

    ros::param::param<double>("~loop_rate", loop_rate, 5.0);

    map_pub = n.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

    map_level_0_sub = n.subscribe("map_level_0", 1, &ChangeMap::mapLevelZeroCallback, this);
    map_level_1_sub = n.subscribe("map_level_1", 1, &ChangeMap::mapLevelOneCallback, this);

    change_map_server = n.advertiseService("change_map", &ChangeMap::changeMapCallback, this);
}

void ChangeMap::mapLevelZeroCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    map_level_0 = *msg;
}

void ChangeMap::mapLevelOneCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    map_level_1 = *msg;
}

bool ChangeMap::changeMapCallback(multi_floor_nav::IntTrigger::Request &req, 
                                  multi_floor_nav::IntTrigger::Response &res){
    if(req.req_int == 0){
        map.header = map_level_0.header;
        map.info = map_level_0.info;
        map.data = map_level_0.data;
    }
    else{
        map.header = map_level_1.header;
        map.info = map_level_1.info;
        map.data = map_level_1.data;
    }
    map_pub.publish(map);

    res.success = true;
    res.message = "Successfully loaded map for level: " + to_string(req.req_int);

    return true;  
}

double ChangeMap::getLoopRate(){
    return loop_rate;
}

int main(int argc, char** argv) {   
    ros::init(argc, argv, "change_map_node");
    ros::NodeHandle n;

    ChangeMap change_map;
    change_map.initialize(n);
    double rate = change_map.getLoopRate();
    if(rate <= 0.0){
        rate = 1.0;
    }
    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return (0);
}

