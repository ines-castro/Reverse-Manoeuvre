#include "reverse_manoeuvre.h"

namespace csai
{
    ReverseManoeuvre::~ReverseManoeuvre()
    {
        // The destructor has to be inside the namespace
    }

    ReverseManoeuvre::ReverseManoeuvre(ros::NodeHandle &f_nh, ros::NodeHandle &f_nhPriv)
        : m_tfListener(m_tfBuffer),         // Initialize listener with buffer
          m_nh(f_nh),                       // Store the node handle
          m_nhPriv(f_nhPriv),               // Store the private node handle
          m_cartDimensionsLoaded(false),    // Cart dimensions not loaded yet
          m_prevClosestIndex(0)             // Start searching from beginning
    {
        // --- NODE PARAMETERS ------------------------------------
        f_nhPriv.param("reverse_speed", m_reverseSpeed, -0.1f);
        f_nhPriv.param("lookahead_distance", m_lookaheadDist, 1.0f);
        f_nhPriv.param("goal_tolerance", m_goalTolerance, 0.1f);
        f_nhPriv.param("gripper_length", m_gripperLength, 0.45f);
        f_nhPriv.param("initial_angle", m_initialOrientation, 37.0f);
        f_nhPriv.param("debug", m_debug, false);
        
        // Target position parameters
        double target_x, target_y;
        f_nhPriv.param("target_x", target_x, -3.0);
        f_nhPriv.param("target_y", target_y, -3.0);
        
        m_initialOrientation = m_initialOrientation * DEG_TO_RAD;       // Convert from degrees to radians

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
        m_cancelSub = f_nh.subscribe("cancel", 1, &ReverseManoeuvre::cancelCb, this);
        m_startSub = f_nh.subscribe("reverse_cmd", 1, &ReverseManoeuvre::startCb, this);

        // --- TIMERS ------------------------------------
        m_tfTimer = f_nh.createTimer(ros::Duration(0.1), &ReverseManoeuvre::robotTfCb, this);
        m_tfTimer.stop(); // Don't start until cart dimensions are loaded

        m_controlTimer = f_nh.createTimer(ros::Duration(0.1), &ReverseManoeuvre::pathFollowing, this);
        m_controlTimer.stop();

        // --- PUBLISHERS ------------------------------------
        m_cmdPub = f_nh.advertise<geometry_msgs::Twist>("cmd_vel", 0);
        m_statusPub = f_nh.advertise<reverse_manoeuvre::ReverseStatus>("status", 10);
        m_cartWheelbasePub = f_nh.advertise<std_msgs::Float64>("cart_wheelbase", 1, true);                // To not get it sfrom ioboard sim
        m_debugPub = f_nh.advertise<std_msgs::Float32MultiArray>("debug_values", 10);
        m_lookaheadMarkerPub = f_nh.advertise<visualization_msgs::Marker>("lookahead_marker", 10);
        m_pathAnglePub = f_nh.advertise<geometry_msgs::PoseStamped>("path_angle", 1);
        m_headingPub = f_nh.advertise<geometry_msgs::PoseStamped>("heading", 1);

        // Latched topic sends it to new subscribers
        m_pathPub = f_nh.advertise<nav_msgs::Path>("reference_path", 1, true);

        publishVelocityCommand(0.0, 0.0); // Ensure robot is stopped at startup
        m_lookaheadDist = 1.7f;
    }

    // ==========================================================
    // TF OPERATIONS 
    // ==========================================================
    void ReverseManoeuvre::robotTfCb(const ros::TimerEvent &event)
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

    void ReverseManoeuvre::updatePoseFromTF(const geometry_msgs::TransformStamped &transform, State &state)
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
                transform.transform.rotation.w);
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

    void ReverseManoeuvre::updateCartPoses()
    {
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
    }

    // ==========================================================
    // REQUIRED INFORMATION (comes from elsewhere)
    // ==========================================================
    bool ReverseManoeuvre::generateReversePath(const State& initialState, const Point2D& target)
    {
        // Clear any existing path and reset search index
        m_referencePath.clear();
        m_prevClosestIndex = 0;  
        
        // Check if cart dimensions are loaded
        if (!m_cartDimensionsLoaded)
        {
            ROS_ERROR("Cannot generate path: Cart dimensions not loaded yet");
            return false;
        }
        
        // Calculate turning radius from cart dimensions
        float turning_radius = 1.5;
        
        // Convert State to RobotState for geometry solver
        ROS_WARN("Initial heading: %f degrees   ", initialState.heading * 180.0 / M_PI);
        RobotState solverState(initialState.x, initialState.y, initialState.heading);
        
        // Create geometry solver and generate path
        GeometrySolver solver(solverState, target, turning_radius);
        solver.setDebug(m_debug);
        
        // Generate waypoints with 5cm spacing
        std::vector<Point2D> waypoints = solver.generatePath(0.05);
        
        if (waypoints.empty())
        {
            ROS_ERROR("Failed to generate reverse path");
            return false;
        }
        
        // Convert Point2D waypoints to PathPoint format 
        m_referencePath.assign(waypoints.begin(), waypoints.end());
        
        // Only log once, with all important info on one line
        if (m_debug)
        {
            ROS_INFO("Path generated");
        }
        
        // Set the target to the last point in the path
        m_target = m_referencePath.back();
        
        visualisePath(m_referencePath);
        
        return true;
    }

    void ReverseManoeuvre::gripperAngleCb(const std_msgs::Float32::ConstPtr &msg)
    {
        m_gripperAngle = msg->data;
    }

    void ReverseManoeuvre::payloadIdCb(const movai_common::PayloadInfo::ConstPtr &msg)
    {
        if (msg->id_candidates.empty())
        {
            return;
        }

        // Change only if the payload ID is different from the current one
        std::string new_payload_id = msg->id_candidates[0];
        if (new_payload_id != m_payloadId)
        {
            if (m_debug)
                ROS_INFO("Detected new payload ID: %s", new_payload_id.c_str());
            m_payloadId = new_payload_id;
            
            // Start TF timer after first successful cart dimensions load
            if(loadCartDimensions(m_payloadId) and !m_tfTimer.hasStarted()){
                m_tfTimer.start();
                if (m_debug)
                    ROS_INFO("TF timer started after loading cart dimensions");
            }

        }
    }

    bool ReverseManoeuvre::loadCartDimensions(const std::string &payload_id)
    {
        // If data loaded is not a struct, there is a problem somewhere
        if (!m_nhPriv.getParam("payload_config", m_payloadConfig) or m_payloadConfig.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_WARN("Config loaded has to be a struct! Current type: %d", m_payloadConfig.getType());
            return false;
        }

        try
        {
            if (m_payloadConfig.hasMember(payload_id))
            {
                // Struct with all the info regarding the payload
                XmlRpc::XmlRpcValue &payload_data = m_payloadConfig[payload_id];
                if (!payload_data.hasMember("object_dimensions") || payload_data.getType() != XmlRpc::XmlRpcValue::TypeStruct)
                {
                    ROS_ERROR("The payload configuration has to have key 'object_dimensions'");
                    return false;
                }

                // Struct with payload dimensions
                XmlRpc::XmlRpcValue &cart_dimensions = payload_data["object_dimensions"];
                if (!cart_dimensions.hasMember("length_to_fixed_wheel") || !cart_dimensions.hasMember("width"))
                {
                    ROS_ERROR("The payload configuration is missing required cart dimension parameters!");
                    return false;
                }

                m_cartLength = (double)(cart_dimensions["length"]);
                m_fixedWheelDist = (double)(cart_dimensions["length_to_fixed_wheel"]);

                m_cartDimensionsLoaded = true;

                // TODO: only because twist to tricycle needs it
                std_msgs::Float64 msg;
                msg.data = m_gripperLength + m_fixedWheelDist;
                m_cartWheelbasePub.publish(msg);

                if (m_debug)
                    ROS_INFO("SUCCESS! Loaded: length=%.3f, wheel_dist=%.3f", m_cartLength, m_fixedWheelDist);
                
                return true;
            }
            else
            {
                ROS_WARN("Configuration file doesn't have information regarding payload with id '%s'", payload_id.c_str());
                return false;
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Exception: %s", e.what());
            return false;
        }
    }

    void ReverseManoeuvre::cancelCb(const std_msgs::Empty::ConstPtr &msg)
    {
        // When cancel is requested
        publishStatus(Status::CANCELLED, "Reverse manoeuvre cancelled by user.");
        if (m_debug)
            ROS_WARN("Cancel request received.");
        publishVelocityCommand(0.0, 0.0); // Stop the robot immediately
        m_prevClosestIndex = 0;  // Ready for next run
        m_controlTimer.stop();
        
        // Clear all visualizations in RViz
        clearAll();
    
    }

    void ReverseManoeuvre::startCb(const std_msgs::String::ConstPtr &msg)
    {
        float target_angle = 0.0;
        ROS_ERROR("Received start command with data: %s", msg->data.c_str());
        // The message received contains start and end point
        try 
        {
            // Parse the string into a JSON object
            auto j = nlohmann::json::parse(msg->data);

            // Check if it's a Start command
            if (j.contains("target") && j["target"].is_array())
            {
                m_target.x = j["target"][0];
                m_target.y = j["target"][1];
                target_angle = j["target"][2];
            }

            if (j.contains("entry_point") && j["entry_point"].is_array())
            {
                m_entryPoint.x = j["entry_point"][0];
                m_entryPoint.y = j["entry_point"][1];
            }

            ROS_WARN("Start command received. Target: (%.2f, %.2f) | TIMER STARTED: %s", m_target.x, m_target.y, m_controlTimer.hasStarted() ? "YES" : "NO");

            // Start the control loop if not already started
            if (!m_controlTimer.hasStarted() and m_cartDimensionsLoaded)
            {
                // Get the latest cart poses before starting the control loop
                updateCartPoses();

                // Convert PathPoint target to Point2D
                Point2D targetPoint(m_target.x, m_target.y, target_angle);

                // Generate reverse path
                if (generateReversePath(m_cartWheelsState, targetPoint))
                {
                    publishStatus(Status::RUNNING, "Reverse manoeuvre started.");
                    m_controlTimer.start();
                    if (m_debug)
                        ROS_INFO("Successfully calculated reverse path. Control timer started.");
                } else 
                {
                    publishStatus(Status::FAILED, "Failed to generate reverse path.");
                }
            }
            // The control loop needs the cart dimensions
            else if (!m_cartDimensionsLoaded)
            {
                ROS_WARN("Cannot start control loop: Cart dimensions not loaded yet!");
            }
           
        }
        catch (nlohmann::json::exception& e)
        {
            ROS_ERROR("Failed to parse reverse_info string: %s", e.what());
        }

    }

    // ==========================================================
    // VISUALISATION AND DEBUGGING
    // ==========================================================

    void ReverseManoeuvre::publishDebugValues(float value1, float value2, float value3)
    {
        std_msgs::Float32MultiArray debugMsg;
        debugMsg.data.resize(3);
        debugMsg.data[0] = value1;
        debugMsg.data[1] = value2;
        debugMsg.data[2] = value3;
        m_debugPub.publish(debugMsg);
    }

    void ReverseManoeuvre::visualizeDebugPose(const PathPoint &target, float angle, ros::Publisher &pub)
    {
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.frame_id = m_worldFrame;
        poseMsg.header.stamp = ros::Time::now();

        // Position of the lookahead target
        poseMsg.pose.position.x = target.x;
        poseMsg.pose.position.y = target.y;
        poseMsg.pose.position.z = 0.0;

        // Convert yaw angle to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, angle);
        poseMsg.pose.orientation = tf2::toMsg(q);

        pub.publish(poseMsg);
    }

    void ReverseManoeuvre::clearDebugPose(ros::Publisher &pub)
    {
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.frame_id = m_worldFrame;
        poseMsg.header.stamp = ros::Time::now();

        // Publish pose at origin with identity orientation to clear the visualization
        poseMsg.pose.position.x = 0.0;
        poseMsg.pose.position.y = 0.0;
        poseMsg.pose.position.z = 0.0;
        poseMsg.pose.orientation.w = 1.0;

        pub.publish(poseMsg);
    }

    void ReverseManoeuvre::visualizeLookaheadMarker(const PathPoint &point, int action)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = m_worldFrame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "lookahead_point";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = action;

        // Position
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = 0.0;

        // Appearance (Bright Green Sphere)
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.380;
        marker.color.g = 0.643;
        marker.color.b = 0.678;

        m_lookaheadMarkerPub.publish(marker);
    }

    void ReverseManoeuvre::visualisePath(const std::vector<PathPoint> &path)
    {
        nav_msgs::Path pathMsg;
        pathMsg.header.frame_id = m_worldFrame;
        pathMsg.header.stamp = ros::Time::now();

        for (size_t i = 0; i < path.size(); i++)
        {
            const PathPoint &point = path[i];

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

    void ReverseManoeuvre::clearAll()
    {
        // Clear all visualizations in RViz
        m_referencePath.clear();
        visualisePath(m_referencePath);  
        PathPoint dummy_point(0.0, 0.0);  
        visualizeLookaheadMarker(dummy_point, visualization_msgs::Marker::DELETE);
        clearDebugPose(m_pathAnglePub);
        clearDebugPose(m_headingPub);
    }

    // ==========================================================
    // COMMAND PUBLISHERS
    // ==========================================================
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

    void ReverseManoeuvre::publishStatus(Status status, const std::string& error_description)
    {
        // Log type according to severity of the status
        if (status == Status::FAILED || status == Status::CANCELLED){
            ROS_ERROR("%s", error_description.c_str());
        } else if (m_debug) {
            ROS_INFO("%s", error_description.c_str());
        }

        // Send the message to the handler
        reverse_manoeuvre::ReverseStatus statusMsg;
        statusMsg.status = statusToString(status);
        statusMsg.error_description = error_description;
        m_statusPub.publish(statusMsg);
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
    
    int ReverseManoeuvre::findClosestPathPoint(State cartState)
    {
        int closestIndex = m_prevClosestIndex;
        float minDistance = std::numeric_limits<float>::max(); // Initialised to inf

        // Start searching from the last closest index for efficiency
        for (int i = m_prevClosestIndex; i < m_referencePath.size(); ++i)
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

        // Update last closest index for next search
        m_prevClosestIndex = closestIndex;

        return closestIndex;
    }

    float ReverseManoeuvre::purePursuitControl(const State &control_point)
    {
        // Obtain the point which has the minimum distance to the control position
        int min_index = findClosestPathPoint(control_point);
        PathPoint closest_point = m_referencePath[min_index];
        PathPoint local_target = closest_point;

        // Search for the point in the path that is at least lookahead_distance away from the closest point
        int i;
        for (i = min_index; i < m_referencePath.size(); ++i)
        {
            float dist = std::sqrt(std::pow(m_referencePath[i].x - control_point.x, 2) +
                                   std::pow(m_referencePath[i].y - control_point.y, 2));
            if (dist >= m_lookaheadDist)
            {
                local_target = m_referencePath[i];
                break;
            }
        }

        // If we reached the end of the path without hitting the lookahead distance, aim for the target point 
        if (i == m_referencePath.size())
        {
            local_target = m_referencePath.back(); 
        }

        // Where the cart needs to face to hit the lookahead point
        float dx = local_target.x - control_point.x;
        float dy = local_target.y - control_point.y;
        float pathAngle = std::atan2(-dy, -dx);

        // Visualisation in Rviz
        visualizeLookaheadMarker(local_target);
        visualizeDebugPose(local_target, pathAngle, m_pathAnglePub);
        PathPoint cp_as_point;
        cp_as_point.x = control_point.x;
        cp_as_point.y = control_point.y;
        visualizeDebugPose(cp_as_point, control_point.heading, m_headingPub);

        // Pure Pursuit control law: κ = 2sin(α)/L where α is heading error, L is lookahead distance
        float headingError = normalizeAngle(pathAngle - control_point.heading);
        float kappa = -2.0f * std::sin(headingError) / m_lookaheadDist;

        ROS_INFO("Path Angle: %.2f | Current Heading: %.2f | Error: %.2f", 
                pathAngle, control_point.heading, headingError);

        // Values for rqt plot
        publishDebugValues(pathAngle, kappa, headingError);

        return kappa;
    }


    void ReverseManoeuvre::pathFollowing(const ros::TimerEvent &event)
    {
        // ----------- Get current cart pose from TF -----------
        updateCartPoses();

        // ----------- Calculate reference heading -----------
        float curvature = purePursuitControl(m_cartWheelsState);
        float w = m_reverseSpeed * curvature; // Pure pursuit control law for angular velocity
        ROS_WARN("Curvature: %.3f | Angular Velocity: %.3f", curvature, w);

        // ----------- Execution and stop conditions -----------
        float dx_target = m_target.x - m_cartBackState.x;
        float dy_target = m_target.y - m_cartBackState.y;
        float distanceToTarget = std::sqrt(dx_target * dx_target + dy_target * dy_target);

        if (distanceToTarget < m_goalTolerance and fabs(m_gripperAngle) < 7.0f) // Check if close enough to target position and gripper is closed
        {
            publishStatus(Status::SUCCESS, "Reverse manoeuvre completed successfully.");
            publishVelocityCommand(0.0, 0.0);
            m_controlTimer.stop();
            return;
        }

        publishVelocityCommand(m_reverseSpeed, w);
    }

} // namespace csai