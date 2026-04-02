#ifndef REVERSE_MANOEUVRE_H
#define REVERSE_MANOEUVRE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>      
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
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

struct State
{
    float x;
    float y;
    float heading;
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

        // --------------------------------
        // ROS COMMUNICATION
        // --------------------------------
        ros::NodeHandle nh_;
        ros::NodeHandle nhPriv_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_; 
        tf2_ros::TransformBroadcaster tfBroadcaster_;
        ros::Subscriber m_odomSub;
        ros::Subscriber m_gripperAngleSub;
        ros::Subscriber m_robotTfSub;
        ros::Timer m_timer;
        ros::Publisher m_cmdPub;

        // --------------------------------
        // CONFIGURATION
        // --------------------------------
        const float m_cartLength;
        const float m_fixedWheelDist, m_gripperLength;
        const float m_reverseSpeed;
        const std::string m_robotFrame;
        const std::string m_gripperFrame;
        const std::string m_cartBackFrame;
        const std::string m_cartWheelsFrame;

        // --------------------------------
        // PARAMETERS
        // --------------------------------
        ros::Time m_lastTime;
        std::vector<PathPoint> ref_path_;

        // Controller parameters
        float K_dist, K_turn, K_hitch;
        float horizon, lookaheadDist;

        float m_gripperAngle;
        State m_robotPose;
        State m_cartState;

        bool m_debug;

        
        // --------------------------------
        // CALLBACKS
        // --------------------------------
        void gripperAngleCb(const std_msgs::Float64::ConstPtr& msg);
        void robotTfCb(const ros::TimerEvent& event);

        /**
        * @brief Loads reference path from CSV file
        * Reads a CSV file with format: x,y (skips header row)
        * and populates m_referencePath with PathPoint structs.
        * 
        * @param file_path Absolute or relative path to CSV file
        */
        void loadCsvPath(const std::string file_path);
        void sendOffsetTF(const std::string parent_frame, const std::string child_frame, float x_offset);
        int findClosestPathPoint();
    

    };

} 
#endif