#ifndef REVERSE_MANOEUVRE_H
#define REVERSE_MANOEUVRE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

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
        tf2_ros::Buffer m_tfBuffer;
        tf2_ros::TransformListener m_tfListener; 
        tf2_ros::TransformBroadcaster m_tfBroadcaster;
        ros::Subscriber m_gripperAngleSub;
        ros::Timer m_tfTimer;
        ros::Timer m_controlTimer;
        ros::Publisher m_cmdPub;

        // ================================
        // Configuration
        // ================================
        float m_reverseSpeed;
        float m_maxGamma; 

        // Cart dimensions
        float m_cartLength, m_fixedWheelDist, m_gripperLength;

        // Controller parameters
        float K_dist, K_turn, K_hitch, horizon, lookaheadDist;

        // Frame names
        std::string m_worldFrame, m_robotFrame, m_gripperFrame, m_cartBackFrame, m_cartWheelsFrame;
        PathPoint m_target;

        // ================================
        // PARAMETERS
        // ================================
        ros::Time m_lastTime;
        std::vector<PathPoint> m_referencePath;

        // Controller parameters

        float m_gripperAngle;
        State m_robotState;
        State m_cartWheelsState;
        State m_cartBackState;

        bool m_debug;
        
        // ================================
        // FUNCTIONS
        // ================================
        void gripperAngleCb(const std_msgs::Float64::ConstPtr& msg);
        void publishVelocityCommand(float linear_x, float angular_z);
        void controlLoop(const ros::TimerEvent& event);
        void robotTfCb(const ros::TimerEvent& event);
        void sendOffsetTF(const std::string parent_frame, const std::string child_frame, float x_offset);
        void updatePoseFromTF(const geometry_msgs::TransformStamped& transform, State& state);
        void loadCsvPath(const std::string file_path);
        int findClosestPathPoint(State cartState);

    };

} 
#endif