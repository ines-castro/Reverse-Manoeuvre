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
          m_state(ManeuverState::REVERSING) // Initialize state to IDLE
    {
        // --- NODE PARAMETERS ------------------------------------
        f_nhPriv.param("reverse_speed", m_reverseSpeed, -0.1f);
        f_nhPriv.param("lookahead_distance", m_lookaheadDist, 1.0f);
        f_nhPriv.param("goal_tolerance", m_goalTolerance, 0.1f);
        f_nhPriv.param("gripper_length", m_gripperLength, 0.45f);
        f_nhPriv.param("initial_angle", m_initialOrientation, 37.0f);
        f_nhPriv.param("debug", m_debug, false);
        
        m_initialOrientation = m_initialOrientation * DEG_TO_RAD;       // Convert from degrees to radians
        m_prevClosestIndex = 0;                                         // Start searching for closest point from the beginning of the path

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
        // m_controlTimer.stop();

        // --- PUBLISHERS ------------------------------------
        m_cmdPub = f_nh.advertise<geometry_msgs::Twist>("cmd_vel", 0);
        m_cartWheelbasePub = f_nh.advertise<std_msgs::Float64>("cart_wheelbase", 1, true);                // To not get it sfrom ioboard sim
        m_debugPub = f_nh.advertise<std_msgs::Float32MultiArray>("debug_values", 10);
        m_lookaheadMarkerPub = f_nh.advertise<visualization_msgs::Marker>("lookahead_marker", 10);
        m_pathAnglePub = f_nh.advertise<geometry_msgs::PoseStamped>("path_angle", 1);
        m_headingPub = f_nh.advertise<geometry_msgs::PoseStamped>("heading", 1);

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

        ROS_INFO("Successfully loaded %d path points from CSV: %s", point_count, file_path.c_str());
        file.close();
    }

    void ReverseManoeuvre::gripperAngleCb(const std_msgs::Float32::ConstPtr &msg)
    {
        m_gripperAngle = msg->data;
    }

    void ReverseManoeuvre::payloadIdCb(const movai_common::PayloadInfo::ConstPtr &msg)
    {
        if (msg->id_candidates.empty())
        {
            if (m_debug)
                ROS_WARN("Received PayloadInfo but id_candidates is empty!");
            return;
        }

        // Change only if the payload ID is different from the current one
        std::string new_payload_id = msg->id_candidates[0];
        if (new_payload_id != m_payloadId)
        {
            if (m_debug)
                ROS_INFO("Detected new payload ID: %s", new_payload_id.c_str());
            m_payloadId = new_payload_id;
            loadCartDimensions(m_payloadId);

            // Start TF timer after first successful cart dimensions load
            if (m_cartDimensionsLoaded && !m_tfTimer.hasStarted())
            {
                m_tfTimer.start();
                m_state = ManeuverState::POSITIONING; // Transition to positioning state
                if (m_debug)
                    ROS_INFO("TF timer started after loading cart dimensions");
            }
        }
    }

    void ReverseManoeuvre::loadCartDimensions(const std::string &payload_id)
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
                XmlRpc::XmlRpcValue &payload_data = m_payloadConfig[payload_id];
                if (!payload_data.hasMember("object_dimensions") || payload_data.getType() != XmlRpc::XmlRpcValue::TypeStruct)
                {
                    ROS_ERROR("The payload configuration has to have key 'object_dimensions'");
                    return;
                }

                // Struct with payload dimensions
                XmlRpc::XmlRpcValue &cart_dimensions = payload_data["object_dimensions"];
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

                if (m_debug)
                    ROS_INFO("SUCCESS! Loaded: length=%.3f, wheel_dist=%.3f", m_cartLength, m_fixedWheelDist);
            }
            else
            {
                ROS_WARN("Configuration file doesn't have information regarding payload with id '%s'", payload_id.c_str());
            }
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Exception: %s", e.what());
        }
    }

    void ReverseManoeuvre::triggerCb(const std_msgs::Bool::ConstPtr &msg)
    {
        // When trigger is requested
        if (msg->data)
        {
            // Start the control loop if not already started
            if (!m_controlTimer.hasStarted() and m_cartDimensionsLoaded)
            {
                m_controlTimer.start();
                if (m_debug)
                    ROS_INFO("Control timer started");
            }
            // The control loop needs the cart dimensions
            else if (!m_cartDimensionsLoaded)
            {
                ROS_WARN("Cannot start control loop: Cart dimensions not loaded yet!");
            }
        }
        else
        {
            if (m_debug)
                ROS_WARN("Trigger to stop received.");
            publishVelocityCommand(0.0, 0.0); // Stop the robot immediately
            m_controlTimer.stop();
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

    void ReverseManoeuvre::visualizeLookaheadMarker(const PathPoint &point)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = m_worldFrame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "lookahead_point";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

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
            float dist = std::sqrt(std::pow(m_referencePath[i].x - closest_point.x, 2) +
                                   std::pow(m_referencePath[i].y - closest_point.y, 2));
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

        // ----------- Execution and stop conditions -----------
        float dx_target = m_target.x - m_cartBackState.x;
        float dy_target = m_target.y - m_cartBackState.y;
        float distanceToTarget = std::sqrt(dx_target * dx_target + dy_target * dy_target);

        if (distanceToTarget < m_goalTolerance and fabs(m_gripperAngle) < 3.0f) // Check if close enough to target position and gripper is closed
        {
            ROS_INFO("Reverse manoeuvre completed successfully.");
            publishVelocityCommand(0.0, 0.0);
            m_controlTimer.stop();
            return;
        }

        publishVelocityCommand(m_reverseSpeed, w);
    }

    // ==========================================================
    // STATE MACHINE (probably gonna dissapear when feature is complete)
    // ==========================================================
    void ReverseManoeuvre::maneuverStages(const ros::TimerEvent &event)
    {
        updateCartPoses(); // Update cartWheelsState with current position

        switch (m_state)
        {
        case ManeuverState::IDLE:
            // Do nothing, waiting for trigger
            break;

        case ManeuverState::POSITIONING:
        {
            // Monitor when the robot reaches the start position of the path

            // Calculate distance from robot to start point
            float dx = m_referencePath[0].x - m_cartWheelsState.x;\
            float dy = m_referencePath[0].y - m_cartWheelsState.y;
            float distanceToStart = std::sqrt(dx * dx + dy * dy);

            // Check if close enough to start position (within 0.1 meters)
            if (distanceToStart < 0.5) // Also check that gripper is closed to avoid false positives
            {
                ROS_INFO("Reached start position. Transitioning to ALIGNING");
                publishVelocityCommand(0.0, 0.0);
                m_state = ManeuverState::REVERSING;
            }
        }
        break;

        case ManeuverState::ALIGNING:
        {
            publishVelocityCommand(0.0, -1.0); // Rotate in place at a fixed speed for testing
            ROS_WARN("Aligning... Current heading: %.2f degrees", m_cartWheelsState.heading * 180.0f / M_PI);
            // float orientationError = normalizeAngle(m_robotState.heading - m_initialOrientation);
            float orientationError = normalizeAngle(m_cartWheelsState.heading - 1.57f);
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

} // namespace csai