#include "ros/ros.h"
#include <ros/package.h>
#include "reverse_manoeuvre.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "reverse_cpp_node");
    ros::NodeHandle nh("~");
    ros::NodeHandle nhPriv("~");

    csai::ReverseManoeuvre reverse_man(nh, nhPriv);

    ros::spin();
    
    return 0;
}