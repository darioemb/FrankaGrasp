#include "grasp_node.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_panda");
    ros::NodeHandle nh("~");

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    while (ros::ok())
    {

        ros::spinOnce();
    }

    return 0;
}