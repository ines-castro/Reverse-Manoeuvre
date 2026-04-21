#ifndef REVERSE_MANOEUVRE_H
#define REVERSE_MANOEUVRE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <ros/package.h>
#include <movai_common/SceneDataArray.h>
#include <movai_common/PayloadInfo.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>

constexpr float DEG_TO_RAD = M_PI / 180.0f;

// States for the reverse maneuver state machine
enum class ManeuverState
{
    IDLE,              // Waiting for trigger
    POSITIONING,       // Moving robot to start position
    ALIGNING,          // Rotating robot to face backward along path
    PICKING,           // Simulate picking up the payload (could be extended to check gripper status)
    REVERSING,         // Actively reversing along path
    COMPLETED          // Reached target
};

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

struct CartDimensions
{
    float length;
    float width;
    float height;
    float length_to_fixed_wheel;
    float attach_offset;
    float engage_bar_height;
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
        ros::NodeHandle m_nh;
        ros::NodeHandle m_nhPriv;
        tf2_ros::Buffer m_tfBuffer;
        tf2_ros::TransformListener m_tfListener; 
        tf2_ros::TransformBroadcaster m_tfBroadcaster;
        ros::Subscriber m_gripperAngleSub;
        ros::Subscriber m_payloadIdSub;
        ros::Subscriber m_triggerSub;
        ros::Timer m_tfTimer;
        ros::Timer m_controlTimer;
        ros::Publisher m_cmdPub;
        ros::Publisher m_pathPub;
        ros::Publisher m_debugPub;
        ros::Publisher m_pathAnglePub;
        ros::Publisher m_headingPub;
        ros::Publisher m_crossTrackError;
        ros::Publisher m_lookaheadMarkerPub;

        // Required for movai twist to tricycle
        ros::Publisher m_cartWheelbasePub;     
        ros::Publisher m_gripperAngleFormatedPub;


        // ================================
        // Configuration
        // ================================
        float m_reverseSpeed;
        float m_maxGamma;
        float m_prevGamma; 
        int m_prevClosestIndex;
        int m_prevLookaheadIndex;

        // Cart dimensions
        float m_cartLength, m_fixedWheelDist, m_gripperLength;

        // Controller parameters
        float m_kDist, m_kTurn, m_kHitch, m_horizon, m_lookaheadDist;

        // Frame names
        std::string m_worldFrame, m_robotFrame, m_gripperFrame, m_cartWheelsFrame, m_cartBackFrame;
        PathPoint m_target;

        // ================================
        // PARAMETERS
        // ================================
        ManeuverState m_state;  // Current state of the maneuver
        ros::Time m_lastTime;
        std::vector<PathPoint> m_referencePath;
        std::string m_payloadId;
        XmlRpc::XmlRpcValue m_payloadConfig;

        // Controller parameters

        float m_gripperAngle;
        State m_robotState;
        State m_cartWheelsState;
        State m_cartBackState;
        float m_initialOrientation;

        bool m_debug;
        bool m_cartDimensionsLoaded;
        
        // ================================
        // FUNCTIONS
        // ================================
        void triggerCb(const std_msgs::Bool::ConstPtr& msg);
        void gripperAngleCb(const std_msgs::Float32::ConstPtr& msg);
        void payloadIdCb(const movai_common::PayloadInfo::ConstPtr& msg);
        void loadCartDimensions(const std::string& payload_id);
        void publishVelocityCommand(float linear_x, float angular_z);
        void publishDebugValues(float value1, float value2, float value3);
        void updatePoses(); 
        void maneuverStages(const ros::TimerEvent& event); 
        float outerLoop(const State& control_point);
        void pathFollowing(const ros::TimerEvent& event);
        void robotTfCb(const ros::TimerEvent& event);
        void sendOffsetTF(const std::string parent_frame, const std::string child_frame, float x_offset);
        void updatePoseFromTF(const geometry_msgs::TransformStamped& transform, State& state);
        void loadCsvPath(const std::string file_path);
        void visualisePath(const std::vector<PathPoint>& path);
        void visualizeDebugPose(const PathPoint& target, float angle, ros::Publisher& pub);
        void visualizeLookaheadMarker(const PathPoint& point);
        int findClosestPathPoint(State cartState);
        float normalizeAngle(float angle);


    };

} 
#endif