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

namespace csai 
{
    class ReverseManoeuvre 
    {
    public:
        // Constructor & Destructor
        ReverseManoeuvre(ros::NodeHandle &f_nh, ros::NodeHandle &f_nhPriv);
        ~ReverseManoeuvre();

    private:
        // ==========================================================
        // ROS COMMUNICATION
        // ==========================================================
        ros::NodeHandle m_nh;
        ros::NodeHandle m_nhPriv;
        
        // TF
        tf2_ros::Buffer m_tfBuffer;
        tf2_ros::TransformListener m_tfListener; 
        tf2_ros::TransformBroadcaster m_tfBroadcaster;
        
        // Subscribers
        ros::Subscriber m_gripperAngleSub;
        ros::Subscriber m_payloadIdSub;
        ros::Subscriber m_triggerSub;
        
        // Timers
        ros::Timer m_tfTimer;
        ros::Timer m_controlTimer;
        
        // Publishers
        ros::Publisher m_cmdPub;
        ros::Publisher m_pathPub;
        ros::Publisher m_debugPub;
        ros::Publisher m_pathAnglePub;
        ros::Publisher m_headingPub;
        ros::Publisher m_lookaheadMarkerPub;
        ros::Publisher m_cartWheelbasePub;     

        // ==========================================================
        // STATE & CONFIGURATION
        // ==========================================================
        ManeuverState m_state;
        bool m_debug;
        bool m_cartDimensionsLoaded;
        
        // Control parameters
        float m_reverseSpeed;
        float m_goalTolerance;
        float m_lookaheadDist;
        float m_initialOrientation;
        
        // Cart dimensions
        float m_cartLength;
        float m_fixedWheelDist;
        float m_gripperLength;
        
        // Frame names
        std::string m_worldFrame;
        std::string m_robotFrame;
        std::string m_gripperFrame;
        std::string m_cartWheelsFrame;
        std::string m_cartBackFrame;
        
        // Payload configuration
        std::string m_payloadId;
        XmlRpc::XmlRpcValue m_payloadConfig;
        
        // Path tracking
        std::vector<PathPoint> m_referencePath;
        PathPoint m_target;
        int m_prevClosestIndex;
        
        // State tracking
        float m_gripperAngle;
        State m_robotState;
        State m_cartWheelsState;
        State m_cartBackState;

        // ==========================================================
        // FUNCTION DECLARATIONS 
        // ==========================================================
        
        // TF Operations
        void robotTfCb(const ros::TimerEvent& event);
        void sendOffsetTF(const std::string parent_frame, const std::string child_frame, float x_offset);
        void updatePoseFromTF(const geometry_msgs::TransformStamped& transform, State& state);
        void updateCartPoses();
        
        // Required Information (callbacks & loading)
        void loadCsvPath(const std::string file_path);
        void gripperAngleCb(const std_msgs::Float32::ConstPtr& msg);
        void payloadIdCb(const movai_common::PayloadInfo::ConstPtr& msg);
        void loadCartDimensions(const std::string& payload_id);
        void triggerCb(const std_msgs::Bool::ConstPtr& msg);
        
        // Visualization & Debugging
        void publishDebugValues(float value1, float value2, float value3);
        void visualizeDebugPose(const PathPoint& target, float angle, ros::Publisher& pub);
        void visualizeLookaheadMarker(const PathPoint& point);
        void visualisePath(const std::vector<PathPoint>& path);
        
        // Command Publishers
        void publishVelocityCommand(float linear_x, float angular_z);
        
        // Path Following Utilities
        float normalizeAngle(float angle);
        int findClosestPathPoint(State cartState);
        float purePursuitControl(const State& control_point);
        void pathFollowing(const ros::TimerEvent& event);
        
        // State Machine
        void maneuverStages(const ros::TimerEvent& event);
    };

} 
#endif