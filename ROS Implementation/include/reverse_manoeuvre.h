#ifndef REVERSE_MANOEUVRE_H
#define REVERSE_MANOEUVRE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

struct PathPoint
{
    float x;
    float y;
};

namespace csai 
{

    class ReverseManoeuvre 
    {
    public:
        // Constructor
        ReverseManoeuvre(ros::NodeHandle &f_nh, ros::NodeHandle &f_nhPriv);
        // Destructor
        ~ReverseManoeuvre();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nhPriv_;

        // --------------------------------
        // PARAMETERS
        // --------------------------------
        float m_gripperAngle, m_reverseSpeed;
        float m_fixedWheelDist, m_gripperLength, m_maxGamma;
        bool m_debug;
        geometry_msgs::Pose m_robotPose;
        std::vector<PathPoint> m_referencePath;

        // --------------------------------
        // SUBSCRIBERS
        // --------------------------------
        ros::Subscriber m_odomSub;
        ros::Subscriber m_gripperAngleSub;
        ros::Timer m_timer;

        // --------------------------------
        // PUBLISHERS
        // --------------------------------
        ros::Publisher m_cmdPub;

        // --------------------------------
        // CALLBACKS
        // --------------------------------
        void loadCsvPath(const std::string file_path);
        void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
        void gripperAngleCb(const std_msgs::Float64::ConstPtr& msg);

    };

} 
#endif