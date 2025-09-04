#include "gnc_functions.hpp"

using namespace std::chrono_literals;

void GNCFunctions::init_publishers_subscribers() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    local_pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
    global_lla_pos_pub = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/mavros/setpoint_position/global", 10);
    global_lla_pos_pub_raw = this->create_publisher<mavros_msgs::msg::GlobalPositionTarget>("/mavros/setpoint_raw/global", 10);
    currentPos = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/global_position/local",
            qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg){this->pose_cb(msg);}
    );
    state_sub = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state",
        qos,
        [this](const mavros_msgs::msg::State::SharedPtr msg){this->state_cb(msg);}
    );
    arming_client = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    land_client = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    takeoff_client = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    command_client = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
    auto_waypoint_pull_client = this->create_client<mavros_msgs::srv::WaypointPull>("/mavros/mission/pull");
    auto_waypoint_push_client = this->create_client<mavros_msgs::srv::WaypointPush>("/mavros/mission/push");
    auto_waypoint_set_current_client = this->create_client<mavros_msgs::srv::WaypointSetCurrent>("/mavros/mission/set_current");
}

geometry_msgs::msg::Point GNCFunctions::enu_2_local(nav_msgs::msg::Odometry current_pose_enu) {
    float x = current_pose_enu.pose.pose.position.x;
    float y = current_pose_enu.pose.pose.position.y;
    float z = current_pose_enu.pose.pose.position.z;
    float deg2rad = (M_PI/180);
    geometry_msgs::msg::Point current_pos_local;
    current_pos_local.x = x*cos((local_offset_g - 90)*deg2rad) - y*sin((local_offset_g - 90)*deg2rad);
    current_pos_local.y = x*sin((local_offset_g - 90)*deg2rad) + y*cos((local_offset_g - 90)*deg2rad);
    current_pos_local.z = z;

    return current_pos_local;
}

void GNCFunctions::pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_g = *msg;
    enu_2_local(current_pose_g);
    float q0 = current_pose_g.pose.pose.orientation.w;
    float q1 = current_pose_g.pose.pose.orientation.x;
    float q2 = current_pose_g.pose.pose.orientation.y;
    float q3 = current_pose_g.pose.pose.orientation.z;
    float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
    //RCLCPP_INFO(this->get_logger(), "Current Heading %f ENU", psi*(180/M_PI));
    //Heading is in ENU
    //IS YAWING COUNTERCLOCKWISE POSITIVE?
    current_heading_g = psi*(180/M_PI) - local_offset_g;
    //RCLCPP_INFO(this->get_logger(), "Current Heading %f origin", current_heading_g);
    //RCLCPP_INFO(this->get_logger(), "x: %f y: %f z: %f", current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, current_pose_g.pose.pose.position.z);
}

geometry_msgs::msg::Point GNCFunctions::get_current_location() {
    geometry_msgs::msg::Point current_pos_local;
	current_pos_local = enu_2_local(current_pose_g);
	return current_pos_local;
}

void GNCFunctions::set_heading(float heading) {
    local_desired_heading_g = heading; 
    heading = heading + correction_heading_g + local_offset_g;
    
    RCLCPP_INFO(this->get_logger(), "Desired Heading %f ", local_desired_heading_g);
    float yaw = heading*(M_PI/180);
    float pitch = 0;
    float roll = 0;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);

    float qw = cy * cr * cp + sy * sr * sp;
    float qx = cy * sr * cp - sy * cr * sp;
    float qy = cy * cr * sp + sy * sr * cp;
    float qz = sy * cr * cp - cy * sr * sp;

    waypoint_g.pose.orientation.w = qw;
    waypoint_g.pose.orientation.x = qx;
    waypoint_g.pose.orientation.y = qy;
    waypoint_g.pose.orientation.z = qz;
}

void GNCFunctions::set_destination(float x, float y, float z, float psi) {
    set_heading(psi);
	//transform map to local
	float deg2rad = (M_PI/180);
	float Xlocal = x*cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*sin((correction_heading_g + local_offset_g - 90)*deg2rad);
	float Ylocal = x*sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*cos((correction_heading_g + local_offset_g - 90)*deg2rad);
	float Zlocal = z;

	x = Xlocal + correction_vector_g.position.x + local_offset_pose_g.x;
	y = Ylocal + correction_vector_g.position.y + local_offset_pose_g.y;
	z = Zlocal + correction_vector_g.position.z + local_offset_pose_g.z;
	RCLCPP_INFO(this->get_logger(), "Destination set to x: %f y: %f z: %f origin frame", x, y, z);

	waypoint_g.pose.position.x = x;
	waypoint_g.pose.position.y = y;
	waypoint_g.pose.position.z = z;

    this->local_pos_pub->publish(waypoint_g);
}

void GNCFunctions::set_destination_lla(float lat, float lon, float alt, float heading) {
    geographic_msgs::msg::GeoPoseStamped lla_msg;
	// mavros_msgs::GlobalPositionTarget 
	lla_msg.pose.position.latitude = lat;
	lla_msg.pose.position.longitude = lon;
	lla_msg.pose.position.altitude = alt;
	float yaw = heading*(M_PI/180);
	float pitch = 0;
	float roll = 0;

	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);
	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);

	float qw = cy * cr * cp + sy * sr * sp;
	float qx = cy * sr * cp - sy * cr * sp;
	float qy = cy * cr * sp + sy * sr * cp;
	float qz = sy * cr * cp - cy * sr * sp;

	lla_msg.pose.orientation.w = qw;
	lla_msg.pose.orientation.x = qx;
	lla_msg.pose.orientation.y = qy;
	lla_msg.pose.orientation.z = qz;
	this->global_lla_pos_pub->publish(lla_msg);
}

void GNCFunctions::set_destination_lla_raw(float lat, float lon, float alt) {
    mavros_msgs::msg::GlobalPositionTarget lla_msg;
	lla_msg.coordinate_frame = lla_msg.FRAME_GLOBAL_TERRAIN_ALT;
	lla_msg.type_mask = lla_msg.IGNORE_VX | lla_msg.IGNORE_VY | lla_msg.IGNORE_VZ | lla_msg.IGNORE_AFX | lla_msg.IGNORE_AFY | lla_msg.IGNORE_AFZ | lla_msg.IGNORE_YAW | lla_msg.IGNORE_YAW_RATE ; 
	lla_msg.latitude = lat;
	lla_msg.longitude = lon;
	lla_msg.altitude = alt;
	// lla_msg.yaw = heading;
	this->global_lla_pos_pub_raw->publish(lla_msg);
}

int GNCFunctions::wait4connect() {
    RCLCPP_INFO(this->get_logger(), "Waiting for FCU connection");
	
    rclcpp::Rate rate(100);
    // wait for FCU connection
	while (rclcpp::ok() && !current_state_g.connected) {
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
	}
    
    if(current_state_g.connected) {
		RCLCPP_INFO(this->get_logger(), "Connected to FCU");	
		return 0;
	} else{
		RCLCPP_INFO(this->get_logger(), "Error connecting to drone");
		return -1;	
	}
}

int GNCFunctions::wait4start() {
    RCLCPP_INFO(this->get_logger(), "Waiting for user to set mode to GUIDED");
	
    rclcpp::Rate rate(100);
    while(rclcpp::ok() && current_state_g.mode != "GUIDED")
	{
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
  	}
  	if(current_state_g.mode == "GUIDED")
	{
		RCLCPP_INFO(this->get_logger(), "Mode set to GUIDED. Mission starting");
		return 0;
	}else{
		RCLCPP_INFO(this->get_logger(), "Error starting mission!!");
		return -1;	
	}
}

int GNCFunctions::initialize_local_frame() {
    //set the orientation of the local reference frame
	RCLCPP_INFO(this->get_logger(), "Initializing local coordinate system");
	local_offset_g = 0;
	
    rclcpp::Rate rate(10);
    for (int i = 1; i <= 30; i++) {
		rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();

		float q0 = current_pose_g.pose.pose.orientation.w;
		float q1 = current_pose_g.pose.pose.orientation.x;
		float q2 = current_pose_g.pose.pose.orientation.y;
		float q3 = current_pose_g.pose.pose.orientation.z;
		float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

		local_offset_g += psi*(180/M_PI);

		local_offset_pose_g.x = local_offset_pose_g.x + current_pose_g.pose.pose.position.x;
		local_offset_pose_g.y = local_offset_pose_g.y + current_pose_g.pose.pose.position.y;
		local_offset_pose_g.z = local_offset_pose_g.z + current_pose_g.pose.pose.position.z;
		// ROS_INFO("current heading%d: %f", i, local_offset_g/i);
	}
	local_offset_pose_g.x = local_offset_pose_g.x/30;
	local_offset_pose_g.y = local_offset_pose_g.y/30;
	local_offset_pose_g.z = local_offset_pose_g.z/30;
	local_offset_g /= 30;
	RCLCPP_INFO(this->get_logger(), "Coordinate offset set");
	RCLCPP_INFO(this->get_logger(), "the X' axis is facing: %f", local_offset_g);
	return 0;
}

int GNCFunctions::arm() {
    set_destination(0,0,0,0);

    rclcpp::Rate rate(100);

	for(int i=0; i<100; i++)
	{
		this->local_pos_pub->publish(waypoint_g);
		rclcpp::spin_some(this->get_node_base_interface());
		rate.sleep();
	}

	// arming
	RCLCPP_INFO(this->get_logger(), "Arming drone");
	
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
	arm_request->value = true;
    
    while (!arming_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Arming client interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(this->get_logger(), "Arming service not available, waiting again...");
    }

    auto result = arming_client->async_send_request(arm_request);
    this->local_pos_pub->publish(waypoint_g);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Arming Successfull");
        return 0;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Arming Failed With %d", result.get()->success);
        return -1;
    }

    return 0;
}

int GNCFunctions::takeoff(float takeoff_alt) {
    set_destination(0,0,takeoff_alt,0);
    
    rclcpp::Rate rate(100);

	for(int i=0; i<100; i++)
	{
		this->local_pos_pub->publish(waypoint_g);
		rclcpp::spin_some(this->get_node_base_interface());
		rate.sleep();
	}

    // Request takeoff

    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    takeoff_request->altitude = takeoff_alt;
    
    while (!takeoff_client->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Takeoff client interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(this->get_logger(), "Takeoff service not available, waiting again...");
    }

    auto takeoff_result = takeoff_client->async_send_request(takeoff_request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), takeoff_result)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Takeoff Sent %d", takeoff_result.get()->success);
        return 0;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Takeoff Failed ");
        return -2;
    }
    
    rclcpp::sleep_for(std::chrono::seconds(2));
    return 0;
}

int GNCFunctions::check_waypoint_reached(float pos_tolerance, float heading_tolerance) {
    this->local_pos_pub->publish(waypoint_g);

    float deltaX = abs(waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x);
    float deltaY = abs(waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y);
    float deltaZ = 0; //abs(waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z);
    float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );

    // RCLCPP_INFO(this->get_logger(), "dMag %f", dMag);
    // RCLCPP_INFO(this->get_logger(), "current pose x %F y %f z %f", (current_pose_g.pose.pose.position.x), (current_pose_g.pose.pose.position.y), (current_pose_g.pose.pose.position.z));
    // RCLCPP_INFO(this->get_logger(), "waypoint pose x %F y %f z %f", waypoint_g.pose.position.x, waypoint_g.pose.position.y,waypoint_g.pose.position.z);
    //check orientation

    float cosErr = cos(current_heading_g*(M_PI/180)) - cos(local_desired_heading_g*(M_PI/180));
    float sinErr = sin(current_heading_g*(M_PI/180)) - sin(local_desired_heading_g*(M_PI/180));
    
    float headingErr = sqrt( pow(cosErr, 2) + pow(sinErr, 2) );

    // RCLCPP_INFO(this->get_logger(), "current heading %f", current_heading_g);
    // RCLCPP_INFO(this->get_logger(), "local_desired_heading_g %f", local_desired_heading_g);
    // RCLCPP_INFO(this->get_logger(), "current heading error %f", headingErr);

    if( dMag < pos_tolerance && headingErr < heading_tolerance){
		return 1;
	} else{
		return 0;
	}
}

int GNCFunctions::set_mode(std::string mode) {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = mode;

    if (!set_mode_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "SetMode service not available");
        return -1;
    }

    auto result_future = set_mode_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (result_future.get()->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "Mode sent to %s", mode.c_str());
            return 0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "SetMode request failed");
            return -1;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "SetMode service call failed");
        return -1;
    }
}

int GNCFunctions::land() {
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

    if (!land_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Land service not available");
        return -1;
    }

    auto result_future = land_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
        == rclcpp::FutureReturnCode::SUCCESS) 
    {
        if (result_future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Landing command set successfully");
            return 0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Landing request failed");
            return -1;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Land service call failed");
        return -1;
    }
}

int GNCFunctions::set_speed(float speed__mps) {
    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->command = 178;
    request->param1 = 1;
    request->param2 = speed__mps;
    request->param3 = -1;
    request->param4 = 0;
	RCLCPP_INFO(this->get_logger(), "Setting speed to %f", speed__mps);

    if (!command_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Command Client service not available");
        return -1;
    }

    auto result_future = command_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
        == rclcpp::FutureReturnCode::SUCCESS) 
    {
        if (result_future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Change speed command request succeeded");
            return 0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Change speed command request failed");
            return -1;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Speed command service call failed");
        return -1;
    }
}

int GNCFunctions::auto_set_current_waypoint(int seq) {
    auto request = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
    request->wp_seq = seq;
    RCLCPP_INFO(this->get_logger(), "Setting current wp to wp # %d", seq);

    if (!auto_waypoint_set_current_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Auto_waypoint_set_current_client service not available");
        return -1;
    }

    auto result_future = auto_waypoint_set_current_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
        == rclcpp::FutureReturnCode::SUCCESS )
    {
        if (result_future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Set current wp request succeeded");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Set current wp request failed");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Set current wp service call failed");
    }

    return 0;
}

int GNCFunctions::set_yaw(float angle, float speed, float dir, float absolute_rel) {
    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->command = 115;
    request->param1 = angle;
    request->param2 = speed;
    request->param3 = dir;
    request->param4 = absolute_rel;
	RCLCPP_INFO(this->get_logger(), "Setting the yaw angle to %f [deg]", angle);

    if (!command_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Command Client service not available");
        return -1;
    }

    auto result_future = command_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
        == rclcpp::FutureReturnCode::SUCCESS) 
    {
        if (result_future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Yaw angle set request succeeded");
            return 0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Yaw angle set request failed");
            return -1;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Yaw angle command service call failed");
        return -1;
    }
};


int GNCFunctions::takeoff_global(float lat, float lon, float alt) {
    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->min_pitch = 0;
    request->latitude = lat;
    request->longitude = lon;
    request->altitude = alt;

    if (!command_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Command Client service not available");
        return -1;
    }

    auto result_future = takeoff_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
        == rclcpp::FutureReturnCode::SUCCESS) 
    {
        if (result_future.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Takeoff sent at the provided GPS coordinates");
            rclcpp::sleep_for(std::chrono::seconds(3));
            return 0;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Takeoff request failed");
            return -1;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Takeoff service call failed");
        return -1;
    }
};
