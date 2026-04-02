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

        // Frame names
        f_nhPriv.param<std::string>("robot_frame", m_worldFrame, std::string("map"));
        f_nhPriv.param<std::string>("robot_frame", m_robotFrame, std::string("base_link"));
        f_nhPriv.param<std::string>("gripper_frame", m_gripperFrame, std::string("gripper_link"));

        // --------------------------------
        // SUBSCRIBERS
        // --------------------------------
        m_odomSub = f_nh.subscribe("odom", 1, &ReverseManoeuvre::odomCb, this);
        m_gripperAngleSub = f_nh.subscribe("gripper_angle", 1, &ReverseManoeuvre::gripperAngleCb, this);
        m_robotTfSub = f_nh.subscribe("tf_in/in", 1, &ReverseManoeuvre::robotTfCb, this);

        // --------------------------------
        // PUBLISHERS
        // --------------------------------
        m_cmdPub = f_nh.advertise<std_msgs::Float64>("m_cmdPub", 0);

        loadCsvPath("./path_points.csv");
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
    void ReverseManoeuvre::robotTfCb(const ros::TimerEvent& event)
    {
        geometry_msgs::TransformStamped baseTF;

        try
        {
            baseTF = m_tfBuffer.lookupTransform(m_worldFrame, m_robotFrame, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Cannot read robot TF: %s", ex.what());
            return;
        }

        // Extract robot position and orientation
        m_robotPose.x = baseTF.transform.translation.x;
        m_robotPose.y = baseTF.transform.translation.y;
        
        tf2::Quaternion q(
            baseTF.transform.rotation.x,
            baseTF.transform.rotation.y,
            baseTF.transform.rotation.z,
            baseTF.transform.rotation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        m_robotPose.theta = (float)yaw;

        // Publish: base link → gripper 
        sendOffsetTF(m_robotPose, m_gripperFrame, -m_gripperLength);

        // Publish: base link → cart_back 
        sendOffsetTF(m_robotFrame, m_cartBackFrame, -m_cartWidth);
    }

    void ReverseManoeuvre::sendOffsetTF(const std::string parent_frame, const std::string child_frame, float x_offset)
    {
        geometry_msgs::TransformStamped newTF;

       newTF.header.stamp = ros::Time::now();
       newTF.header.frame_id = parent_frame;
       newTF.child_frame_id = child_frame;

       // Set the translation offset
       newTF.transform.translation.x = x_offset;
       newTF.transform.translation.y = 0.0;
       newTF.transform.translation.z = 0.0;

       // Set the rotation to identity
       newTF.transform.rotation = tf2::toMsg(tf2::Quaternion::getIdentity());

       tfBroadcaster.sendTransform(newTF);
    }

    void ReverseManoeuvre::gripperAngleCb(const std_msgs::Float64::ConstPtr& msg)
    {
        // Update the gripper angle based on the received message
        m_gripperAngle = msg->data;
    }

    State ReverseManoeuvre::calculateCartState(const float robotPose[3], float hitchAngle)
    {
        State cart;

        // Hitch point at the back of the robot
        float hitchX = robotPose[0] - m_gripperLength * cos(robotPose[2]);
        float hitchY = robotPose[1] - m_gripperLength * sin(robotPose[2]);
        
        // Cart heading = robot heading + hitch angle
        float cartHeading = robotPose[2] + hitchAngle;
        
        // Cart position
        cart.x = hitchX - m_fixedWheelDist * cos(cartHeading);
        cart.y = hitchY - m_fixedWheelDist * sin(cartHeading);
        cart.heading = cartHeading;
        
        return cart;
    }



}