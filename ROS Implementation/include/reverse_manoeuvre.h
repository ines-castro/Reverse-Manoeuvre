#ifndef REVERSE_MANOEUVRE_H
#define REVERSE_MANOEUVRE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <reverse_manoeuvre/ReverseStatus.h>  // Custom message
#include <nlohmann/json.hpp>  // For JSON parsing in startCb
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/Marker.h>
#include <movai_common/PayloadInfo.h>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include "geometry_solver.h"

constexpr float DEG_TO_RAD = M_PI / 180.0f;

enum Status
{
    OFF,
    RUNNING,
    SUCCESS,
    FAILED,
    CANCELLED
};

// Convert Status enum to string for ROS message
inline std::string statusToString(Status status) {
    switch (status) {
        case OFF: return "OFF";
        case RUNNING: return "RUNNING";
        case SUCCESS: return "SUCCESS";
        case FAILED: return "FAILED";
        case CANCELLED: return "CANCELLED";
        default: return "UNKNOWN";
    }
}

struct PathPoint
{
    float x;
    float y;
    
    // Default constructor
    PathPoint() : x(0.0f), y(0.0f) {}
    
    // Constructor from x, y
    PathPoint(float x_, float y_) : x(x_), y(y_) {}
    
    // Constructor from Point2D (for seamless conversion)
    PathPoint(const Point2D& p) : x(p.x), y(p.y) {}
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
        ros::Subscriber m_cancelSub;
        ros::Subscriber m_startSub;
        
        // Timers
        ros::Timer m_tfTimer;
        ros::Timer m_controlTimer;
        
        // Publishers
        ros::Publisher m_cmdPub;
        ros::Publisher m_statusPub;
        ros::Publisher m_pathPub;
        ros::Publisher m_debugPub;
        ros::Publisher m_pathAnglePub;
        ros::Publisher m_headingPub;
        ros::Publisher m_lookaheadMarkerPub;
        ros::Publisher m_cartWheelbasePub;     

        // ==========================================================
        // STATE & CONFIGURATION
        // ==========================================================
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
        PathPoint m_entryPoint;  
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
        bool generateReversePath(const State& initialState, const Point2D& target);
        void gripperAngleCb(const std_msgs::Float32::ConstPtr& msg);
        void payloadIdCb(const movai_common::PayloadInfo::ConstPtr& msg);
        bool loadCartDimensions(const std::string& payload_id);
        void cancelCb(const std_msgs::Empty::ConstPtr &msg);
        void startCb(const std_msgs::String::ConstPtr &msg);
        
        // Visualization & Debugging
        void publishDebugValues(float value1, float value2, float value3);
        void visualizeDebugPose(const PathPoint& target, float angle, ros::Publisher& pub);
        void clearDebugPose(ros::Publisher& pub);
        void visualizeLookaheadMarker(const PathPoint& point, int action = visualization_msgs::Marker::ADD);
        void visualisePath(const std::vector<PathPoint>& path);
        void clearAll();
        
        // Command Publishers
        void publishVelocityCommand(float linear_x, float angular_z);
        void publishStatus(Status status, const std::string& error_description = "");
        
        // Path Following Utilities
        float normalizeAngle(float angle);
        int findClosestPathPoint(State cartState);
        float purePursuitControl(const State& control_point);
        void pathFollowing(const ros::TimerEvent& event);

    };

} 
#endif