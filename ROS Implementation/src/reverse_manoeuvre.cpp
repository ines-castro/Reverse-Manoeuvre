#include "reverse_manoeuvre.h"

namespace csai
{
    ReverseManoeuvre::~ReverseManoeuvre() 
    {
        // The destructor has to be inside the namespace
    }

    ReverseManoeuvre::ReverseManoeuvre(ros::NodeHandle &f_nh, ros::NodeHandle &f_nhPriv) 
    {
        // --------------------------------
        // PARAMETERS
        // --------------------------------
        f_nhPriv.param("gripper_angle", m_gripperAngle, 0.0f);
        f_nhPriv.param("reverse_speed", m_reverseSpeed, -0.2f);
        f_nhPriv.param("fixed_wheel_dist", m_fixedWheelDist, 0.25f);
        f_nhPriv.param("gripper_length", m_gripperLength, 0.45f); // Distance: Robot center to Pivot
        f_nhPriv.param("gripper_angle_limit", m_maxGamma, 37.0f);
        m_maxGamma = m_maxGamma * (M_PI / 180.0f);
        f_nhPriv.param("debug", m_debug, false);

        // --------------------------------
        // SUBSCRIBERS
        // --------------------------------
        m_odomSub = f_nh.subscribe("odom", 1, &ReverseManoeuvre::odomCb, this);
        m_gripperAngleSub = f_nh.subscribe("gripper_angle", 1, &ReverseManoeuvre::gripperAngleCb, this);

        // --------------------------------
        // PUBLISHERS
        // --------------------------------
        m_cmdPub = f_nh.advertise<std_msgs::Float64>("m_cmdPub", 0);
    }

    void ReverseManoeuvre::loadCsvPath(const std::string file_path)
    {
        std::ifstream file(file_path); // Open the CSV file
        std::string line;

        // Skip the header line
        std::getline(file, line);

        while (std::getline(file, line))
        {
            std::stringstream iss(line);
            std::string word; 
            PathPoint point; // Struct to hold parsed details

            // Read x
            std::getline(iss, word, ',');
            point.x = std::stof(word);

            // Read y
            std::getline(iss, word, ',');
            point.y = std::stof(word);
            
            // Add the point to the reference path vector
            m_referencePath.push_back(point);
        }

    }
    // ---------------------------------------------------------
    // CALLBACKS
    // ---------------------------------------------------------
    void ReverseManoeuvre::gripperAngleCb(const std_msgs::Float64::ConstPtr& msg)
    {
        // Update the gripper angle based on the received message
        m_gripperAngle = msg->data;
    }

    void ReverseManoeuvre::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Update the robot's pose based on the received odometry message
        m_robotPose = msg->pose.pose;
    }
}