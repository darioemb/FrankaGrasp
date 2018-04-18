#include "io_haf_msg.hpp"

int main (int argc, char** argv)
{
    ros::init (argc, argv, "node");
    ros::NodeHandle nh("~");
    ros::Subscriber input = nh.subcribe("/calc_grasppoints_svm_action_server/result", 1, &haf_callback);
    ros::spin();
}