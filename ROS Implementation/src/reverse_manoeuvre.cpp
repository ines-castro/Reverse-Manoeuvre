#include "reverse_manoeuvre.h"

namespace csai
{
    ReverseManoeuvre::~ReverseManoeuvre() 
    {
        // The destructor has to be inside the namespace
    }

    ReverseManoeuvre::ReverseManoeuvre(ros::NodeHandle &f_nh, ros::NodeHandle &f_nhPriv) 
        : m_tfListener(m_tfBuffer),     // Initialize listener with buffer
          m_nh(f_nh),                   // Store the node handle
          m_nhPriv(f_nhPriv),           // Store the private node handle
          m_cartDimensionsLoaded(false), // Cart dimensions not loaded yet
          m_state(ManeuverState::REVERSING)   // Initialize state to IDLE
    {
        // --- PARAMETERS ------------------------------------
        f_nhPriv.param("reverse_speed", m_reverseSpeed, -0.2f);
        f_nhPriv.param("gripper_length", m_gripperLength, 0.45f);
        f_nhPriv.param("initial_angle", m_initialOrientation, 37.0f);
        f_nhPriv.param("gripper_angle_limit", m_maxGamma, 37.0f);
        f_nhPriv.param("debug", m_debug, false);
        f_nhPriv.param("K_dist", m_kDist, 8.0f);
        f_nhPriv.param("K_turn", m_kTurn, 1.5f);
        f_nhPriv.param("lookahead_distance", m_lookaheadDist, 0.5f);

        // Convert from degrees to radians
        m_initialOrientation = m_initialOrientation * DEG_TO_RAD;
        m_maxGamma = m_maxGamma * DEG_TO_RAD;
        m_prevGamma = 0.0f; // Initialize previous gamma to zero    
        
        // Frame names
        f_nhPriv.param<std::string>("world_frame", m_worldFrame, std::string("map"));
        f_nhPriv.param<std::string>("robot_frame", m_robotFrame, std::string("base_link"));
        f_nhPriv.param<std::string>("gripper_frame", m_gripperFrame, std::string("gripper_move"));
        f_nhPriv.param<std::string>("cart_back_frame", m_cartBackFrame, std::string("cart_back"));
        f_nhPriv.param<std::string>("cart_wheels_frame", m_cartWheelsFrame, std::string("cart_fixed_wheels"));
        f_nhPriv.param("payload_config", m_payloadConfig, XmlRpc::XmlRpcValue());
        
        // --- SUBSCRIBERS ------------------------------------
        m_gripperAngleSub = f_nh.subscribe("gripper_angle", 1, &ReverseManoeuvre::gripperAngleCb, this);
        m_payloadIdSub = f_nh.subscribe("payload_info", 1, &ReverseManoeuvre::payloadIdCb, this);
        m_triggerSub = f_nh.subscribe("trigger", 1, &ReverseManoeuvre::triggerCb, this);

        // --- TIMERS ------------------------------------
        m_tfTimer = f_nh.createTimer(ros::Duration(0.1), &ReverseManoeuvre::robotTfCb, this);
        m_tfTimer.stop(); // Don't start until cart dimensions are loaded
    
        m_controlTimer = f_nh.createTimer(ros::Duration(0.1), &ReverseManoeuvre::maneuverStages, this);
        //m_controlTimer.stop();

        // --- PUBLISHERS ------------------------------------
        m_cmdPub = f_nh.advertise<geometry_msgs::Twist>("cmd_vel", 0);
        m_crossTrackError = f_nh.advertise<std_msgs::Float32>("crossTrackError", 0);
        m_cartWheelbasePub = f_nh.advertise<std_msgs::Float64>("cart_wheelbase", 1, true);                // To not get it sfrom ioboard sim
        m_gripperAngleFormatedPub = f_nh.advertise<std_msgs::Float64>("gripper_angle_formated", 1, true); // In Float64 instead of Float32
        m_debugPub = f_nh.advertise<std_msgs::Float32MultiArray>("debug_values", 10);

        // Latched topic sends it to new subscribers
        m_pathPub = f_nh.advertise<nav_msgs::Path>("reference_path", 1, true); 

        std::string package_path = ros::package::getPath("reverse_manoeuvre_pkg");
        std::string csv_path = package_path + "/config/path_points.csv";
        loadCsvPath(csv_path);
        m_target = m_referencePath.back();
        visualisePath(m_referencePath);

        publishVelocityCommand(0.0, 0.0); // Ensure robot is stopped at startup
    }

    // ==========================================================
    // TRANSFORM OPERATIONS
    // ==========================================================
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
        updatePoseFromTF(baseTF, m_robotState);

        // Publish TF: gripper_move → fixed_wheels 
        sendOffsetTF(m_gripperFrame, m_cartWheelsFrame, -m_fixedWheelDist); 

        // Publish: gripper_move → cart_back
        sendOffsetTF(m_gripperFrame, m_cartBackFrame, -m_cartLength);
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

       m_tfBroadcaster.sendTransform(newTF);
    }

    void ReverseManoeuvre::updatePoseFromTF(const geometry_msgs::TransformStamped& transform, State& state)
    {
        try
        {
            // Save the position
            state.x = transform.transform.translation.x;
            state.y = transform.transform.translation.y;
            
            // Extract orientation (convert quaternion to yaw angle)
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            
            // Save the heading angle
            state.heading = (float)yaw;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Could not get TF: %s", ex.what());
        }
    }

    // ==========================================================
    // SIMPLE CALLBACKS 
    // ==========================================================
    void ReverseManoeuvre::triggerCb(const std_msgs::Bool::ConstPtr& msg)
    {
        // When trigger is requested
        if (msg->data)
        {   
            // Start the control loop if not already started
            if (!m_controlTimer.hasStarted() and m_cartDimensionsLoaded)
            {
                m_controlTimer.start();
                if (m_debug) ROS_INFO("Control timer started");
            }
            // The control loop needs the cart dimensions
            else if (!m_cartDimensionsLoaded)
            {
                ROS_WARN("Cannot start control loop: Cart dimensions not loaded yet!");
            }
        }
        else
        {
            if (m_debug) ROS_WARN("Trigger to stop received.");
            publishVelocityCommand(0.0, 0.0); // Stop the robot immediately
            m_controlTimer.stop();
        }
    }

    void ReverseManoeuvre::publishVelocityCommand(float linear_x, float angular_z)
    {
        geometry_msgs::Twist cmdMsg;

        // Set linear velocity 
        cmdMsg.linear.x = linear_x;
        cmdMsg.linear.y = 0.0;
        cmdMsg.linear.z = 0.0;
        
        // Set angular velocity 
        cmdMsg.angular.x = 0.0;
        cmdMsg.angular.y = 0.0;
        cmdMsg.angular.z = angular_z;
        
        // Publish the command
        m_cmdPub.publish(cmdMsg);
    }

    void ReverseManoeuvre::publishDebugValues(float gamma_ref, float gamma, float gamma_error)
    {
        std_msgs::Float32MultiArray debugMsg;
        debugMsg.data.resize(3);
        debugMsg.data[0] = gamma_ref;
        debugMsg.data[1] = gamma;
        debugMsg.data[2] = gamma_error;
        m_debugPub.publish(debugMsg);
    }
    
    void ReverseManoeuvre::payloadIdCb(const movai_common::PayloadInfo::ConstPtr& msg)
    {
         if (msg->id_candidates.empty()) 
        {
            if(m_debug) ROS_WARN("Received PayloadInfo but id_candidates is empty!");
            return; 
        }

        // Change only if the payload ID is different from the current one
        std::string new_payload_id = msg->id_candidates[0];
        if (new_payload_id != m_payloadId)
        {
            if (m_debug) ROS_INFO("Detected new payload ID: %s", new_payload_id.c_str());
            m_payloadId = new_payload_id;
            loadCartDimensions(m_payloadId);

            // Start TF timer after first successful cart dimensions load
            if (m_cartDimensionsLoaded && !m_tfTimer.hasStarted())
            {
                m_tfTimer.start();
                m_state = ManeuverState::POSITIONING; // Transition to positioning state
                if (m_debug) ROS_INFO("TF timer started after loading cart dimensions");
            }
        }
    }   

    void ReverseManoeuvre::gripperAngleCb(const std_msgs::Float32::ConstPtr& msg)
    {
        m_gripperAngle = msg->data;
    }

    void ReverseManoeuvre::loadCartDimensions(const std::string& payload_id)
    {
        // If data loaded is not a struct, there is a problem somewhere
        if (!m_nhPriv.getParam("payload_config", m_payloadConfig) or m_payloadConfig.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("Config loaded has to be a struct! Current type: %d", m_payloadConfig.getType());
            return;
        }

        try
        {
            if (m_payloadConfig.hasMember(payload_id))
            {
                // Struct with all the info regarding the payload
                XmlRpc::XmlRpcValue& payload_data = m_payloadConfig[payload_id];
                if (!payload_data.hasMember("object_dimensions") || payload_data.getType() != XmlRpc::XmlRpcValue::TypeStruct)
                {
                    ROS_ERROR("The payload configuration has to have key 'object_dimensions'");
                    return;
                }

                // Struct with payload dimensions
                XmlRpc::XmlRpcValue& cart_dimensions = payload_data["object_dimensions"];
                if (!cart_dimensions.hasMember("length_to_fixed_wheel") || !cart_dimensions.hasMember("width"))
                {
                    ROS_ERROR("The payload configuration is missing required cart dimension parameters!");
                    return;
                }

                m_cartLength = (double)(cart_dimensions["length"]);
                m_fixedWheelDist = (double)(cart_dimensions["length_to_fixed_wheel"]);

                m_cartDimensionsLoaded = true; 

                // TODO: only because twist to tricycle needs it
                std_msgs::Float64 msg;
                msg.data = m_gripperLength + m_fixedWheelDist;
                m_cartWheelbasePub.publish(msg);
                
                if (m_debug) ROS_INFO("SUCCESS! Loaded: length=%.3f, wheel_dist=%.3f", m_cartLength, m_fixedWheelDist);
            }
            else
            {
                ROS_WARN("Configuration file doesn't have information regarding payload with id '%s'", payload_id.c_str());
            }
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Exception: %s", e.what());
        }
    }

    // ==========================================================
    // PATH FOLLOWING UTILITIES
    // ==========================================================
    float ReverseManoeuvre::normalizeAngle(float angle)
    {
        angle = fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0)
            angle += 2.0 * M_PI;
        return angle - M_PI;
    }

    void ReverseManoeuvre::loadCsvPath(const std::string file_path)
    {
        std::ifstream file(file_path); // Open the CSV file
        // Verify that the file can be opened
        if (!file.is_open())
        {
            ROS_ERROR("Failed to open CSV file: %s", file_path.c_str());
            return;
        }
        
        std::string line;
        std::getline(file, line);
        
        int point_count = 0;
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
            point_count++;
        }

        ROS_INFO("Loaded %d path points from CSV", point_count);
        file.close();
    }

    void ReverseManoeuvre::visualisePath(const std::vector<PathPoint>& path)
    {
        nav_msgs::Path pathMsg;
        pathMsg.header.frame_id = m_worldFrame;
        pathMsg.header.stamp = ros::Time::now();

        for (size_t i = 0; i < path.size(); i++)
        {
            const PathPoint& point = path[i];

            geometry_msgs::PoseStamped pose;

            // Fill in the x, y coordinates 
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0; 
            pose.pose.orientation.w = 1.0; // No rotation

            // Add this to the list of poses in the path message
            pathMsg.poses.push_back(pose);
        }

        m_pathPub.publish(pathMsg);
    }

    int ReverseManoeuvre::findClosestPathPoint(State cartState)
    {
        int closestIndex = -1;
        float minDistance = std::numeric_limits<float>::max(); // Initialised to inf

        for (int i = 0; i < m_referencePath.size(); ++i)
        {
            // Calculate Euclidean distance from current cart position to the path point
            float dx = cartState.x - m_referencePath[i].x;
            float dy = cartState.y - m_referencePath[i].y;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance < minDistance)
            {
                minDistance = distance;
                closestIndex = i;
            }
        }

        return closestIndex;
    }

    // ==========================================================
    // MAIN LOOP
    // ==========================================================
    void ReverseManoeuvre::maneuverStages(const ros::TimerEvent& event)
    {
        switch (m_state)
        {
            case ManeuverState::IDLE:
                // Do nothing, waiting for trigger
                break;

            case ManeuverState::POSITIONING:
            {
                // Monitor when the robot reaches the start position of the path
        
                // Calculate distance from robot to start point
                float dx = m_referencePath[0].x - m_robotState.x;
                float dy = m_referencePath[0].y - m_robotState.y;
                float distanceToStart = std::sqrt(dx * dx + dy * dy);

                // Check if close enough to start position (within 0.1 meters)
                if (distanceToStart < 0.1) // Also check that gripper is closed to avoid false positives
                {
                    ROS_INFO("Reached start position. Transitioning to ALIGNING");
                    publishVelocityCommand(0.0, 0.0); 
                    m_state = ManeuverState::ALIGNING; 
                }
            }
            break;

            case ManeuverState::ALIGNING: 
            {
                publishVelocityCommand(0.0, -1.0); // Rotate in place at a fixed speed for testing
                ROS_WARN("Aligning... Current heading: %.2f degrees", m_robotState.heading * 180.0f / M_PI);
                float orientationError = normalizeAngle(m_robotState.heading - m_initialOrientation);
                if (fabs(orientationError) < 0.1) 
                {
                    publishVelocityCommand(0.0, 0.0);
                    ROS_INFO("Aligned with initial angle.");
                    m_state = ManeuverState::PICKING;
                }
            }
            break;

            case ManeuverState::PICKING:
                if (fabs(m_gripperAngle) < 0.5f) // Check if gripper is closed (simulate picking)
                {
                    ROS_INFO("Simulated picking complete. Transitioning to REVERSING.");
                    m_state = ManeuverState::REVERSING;
                }
                break;

            case ManeuverState::REVERSING:
                pathFollowing(event);
                break;

            case ManeuverState::COMPLETED:
                publishVelocityCommand(0.0, 0.0);
                m_controlTimer.stop();
                break;
        }
    }

    float ReverseManoeuvre::outerLoop(const State& control_point)
    {
        // ----------- Find target on path (Pure Pursuit) -----------

        // Obtain the point which has the minimum distance to the robot position
        int min_index = findClosestPathPoint(control_point);
        PathPoint closest_point = m_referencePath[min_index];
        PathPoint local_target = closest_point;

        // Search for the point in the path that is at least lookahead_distance away from the closest point
        for (int i = min_index; i < m_referencePath.size(); ++i)
        {
            float dist = std::sqrt(std::pow(m_referencePath[i].x - closest_point.x, 2) +
                                    std::pow(m_referencePath[i].y - closest_point.y, 2));
            if (dist >= m_lookaheadDist)
            {
                local_target = m_referencePath[i];
                break;
            }
        }

        // Where the cart needs to face to hit the lookahead point
        float dx = local_target.x - control_point.x;
        float dy = local_target.y - control_point.y;
        float pathAngle = std::atan2(dy, dx);

        // dubious
        float headingError = normalizeAngle(control_point.heading - pathAngle);
        float kappa = 2.0f * std::sin(headingError) / m_lookaheadDist;


        // Should i calculate the cross track error or not
        std_msgs::Float32 msg;
        msg.data = headingError;
        m_crossTrackError.publish(msg);

        // Convert to hitch reference
        float gamma_ref = std::atan(1.3 * kappa);

        return std::min(-m_maxGamma, std::max(gamma_ref, m_maxGamma));
    }

    float ReverseManoeuvre::innerLoop(float gamma_ref, float dt)
    {
        ROS_WARN("gamma: %.3f, gamma_ref: %.3f", m_gripperAngle, gamma_ref);
        float gamma = m_gripperAngle * DEG_TO_RAD;
        float gamma_error = normalizeAngle(gamma_ref - gamma);

  
        float gamma_rate = 0.0f;
        if (dt > 1e-3)
            gamma_rate = (gamma - m_prevGamma) / dt;

        m_prevGamma = gamma;

        float k_p = 0.5f;
        float k_d = 0.0f;

        float w = k_p * gamma_error - k_d * gamma_rate;


        ROS_INFO("gamma rate: %.3f, gamma error: %.3f, w: %.3f", gamma_rate, gamma_error, w);


        publishDebugValues(gamma_ref, gamma, gamma_error);

        return w;
    }

    void ReverseManoeuvre::pathFollowing(const ros::TimerEvent& event)
    {
        // ----------- Get current cart pose from TF -----------
        geometry_msgs::TransformStamped cartWheelsTF, cartBackTF;
        try
        {
            cartWheelsTF = m_tfBuffer.lookupTransform(m_worldFrame, m_cartWheelsFrame, ros::Time(0));
            cartBackTF = m_tfBuffer.lookupTransform(m_worldFrame, m_cartBackFrame, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Cannot read cart TF: %s", ex.what());
            return;
        }
        // Update cart variables
        updatePoseFromTF(cartWheelsTF, m_cartWheelsState);
        updatePoseFromTF(cartBackTF, m_cartBackState);
        
        double dt = (event.current_real - event.last_real).toSec();

        // ----------- Calculate reference heading -----------
        //float gamma_ref = outerLoop(m_cartWheelsState);

        float gamma_ref = 0.05;

        // ----------- Calculate control command -----------
        float w = innerLoop(gamma_ref, dt);

        
        // ----------- Execution and stop conditions -----------

        // Check if target reached (use cart back position)
        float dx_target = m_target.x - m_cartBackState.x;
        float dy_target = m_target.y - m_cartBackState.y;
        float distanceToTarget = std::sqrt(dx_target * dx_target + dy_target * dy_target);
        
        if (distanceToTarget < 0.1)
        {
            ROS_INFO("Target reached. Stopping.");
            publishVelocityCommand(0.0, 0.0);
            m_controlTimer.stop();
            return;
        }

        m_reverseSpeed = -0.05; // Set a constant reverse speed for testing

        ROS_WARN("Sending command: linear_x=%.3f, angular_z=%.3f, ", m_reverseSpeed, w);
        publishVelocityCommand(m_reverseSpeed, w);
        
    }

} // namespace csai