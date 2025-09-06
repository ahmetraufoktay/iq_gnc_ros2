#include "gnc_node.hpp"

using namespace std::chrono_literals;

namespace GNC {
    void NodeAPI::init_publishers_subscribers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        
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

    void NodeAPI::processQueue() {
        if (mission_queue.empty()) return;

        Task & task = mission_queue.front();

        if (!task.started) {
            task.on_start();
            task.started = true;
        }

        if (task.done()) {
            mission_queue.pop();
        }
    }

    geometry_msgs::msg::Point NodeAPI::enu_2_local(nav_msgs::msg::Odometry current_pose_enu) {
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

    void NodeAPI::pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
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

    geometry_msgs::msg::Point NodeAPI::get_current_location() {
        geometry_msgs::msg::Point current_pos_local;
        current_pos_local = enu_2_local(current_pose_g);
        return current_pos_local;
    }

    void NodeAPI::set_heading(float heading) {
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

    void NodeAPI::set_destination(float x, float y, float z, float psi) {
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

    void NodeAPI::set_destination_lla(float lat, float lon, float alt, float heading) {
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

    void NodeAPI::set_destination_lla_raw(float lat, float lon, float alt) {
        mavros_msgs::msg::GlobalPositionTarget lla_msg;
        lla_msg.coordinate_frame = lla_msg.FRAME_GLOBAL_TERRAIN_ALT;
        lla_msg.type_mask = lla_msg.IGNORE_VX | lla_msg.IGNORE_VY | lla_msg.IGNORE_VZ | lla_msg.IGNORE_AFX | lla_msg.IGNORE_AFY | lla_msg.IGNORE_AFZ | lla_msg.IGNORE_YAW | lla_msg.IGNORE_YAW_RATE ; 
        lla_msg.latitude = lat;
        lla_msg.longitude = lon;
        lla_msg.altitude = alt;
        // lla_msg.yaw = heading;
        this->global_lla_pos_pub_raw->publish(lla_msg);
    }

    Task NodeAPI::wait4connect() {        
        return {
            [this]() -> bool {
                if (current_state_g.connected) {
                    RCLCPP_INFO(this->get_logger(), "Connected to FCU");
                    return true;
                }
                return false;
            },
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Waiting for FCU connection...");
            }
        };
    }

    Task NodeAPI::wait4start() {
        return {
            [this]() -> bool {
                if (current_state_g.mode == "GUIDED") {
                    RCLCPP_INFO(this->get_logger(), "Mode set to GUIDED. Mission starting");
                    return true;
                }
                return false;
            },
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Waiting for user to set mode to GUIDED");
            }
        };
    }

    Task NodeAPI::initialize_local_frame() {
        return {
            [this]() -> bool {
                static int i = 0;
                static float psi_sum = 0;
                static geometry_msgs::msg::Point pos_sum;
            
                pos_sum.x = pos_sum.y = pos_sum.z = 0;

                if (i >= 30) {
                    local_offset_g = psi_sum / 30.0f;
                    local_offset_pose_g.x = pos_sum.x / 30.0f;
                    local_offset_pose_g.y = pos_sum.y / 30.0f;
                    local_offset_pose_g.z = pos_sum.z / 30.0f;

                    RCLCPP_INFO(this->get_logger(), "Coordinate offset set");
                    RCLCPP_INFO(this->get_logger(), "the X' axis is facing: %f", local_offset_g);
                    return true;
                }

                float q0 = current_pose_g.pose.pose.orientation.w;
                float q1 = current_pose_g.pose.pose.orientation.x;
                float q2 = current_pose_g.pose.pose.orientation.y;
                float q3 = current_pose_g.pose.pose.orientation.z;
                float psi = atan2(2*(q0*q3 + q1*q2), 1 - 2*(pow(q2,2) + pow(q3,2))) * (180/M_PI);

                psi_sum += psi;

                pos_sum.x += current_pose_g.pose.pose.position.x;
                pos_sum.y += current_pose_g.pose.pose.position.y;
                pos_sum.z += current_pose_g.pose.pose.position.z;

                ++i;
                return false; 
            },
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Initializing local coordinate system");
            }
        };
    }
    
    Task NodeAPI::arm() {
        return {
            [this]() ->bool {

                static bool request_sent = false;
                static int publish_count = 0;
                static auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                static rclcpp::Time start;

                req->value = true;

                if (publish_count < 100) {
                    this->local_pos_pub->publish(waypoint_g);
                    publish_count++;
                    return false;
                }

                if (!request_sent) {
                    if (!arming_client->service_is_ready()) {
                        return false;
                    }

                    auto future = arming_client->async_send_request(req);
                    request_sent = true;
                    start = this->now();
                    return false;
                }

                if (current_state_g.armed) {
                    RCLCPP_INFO(this->get_logger(), "Arming Succesfull");
                    return true;
                }

                if ((this->now() - start).seconds() > 10.0) {
                    RCLCPP_ERROR(this->get_logger(), "Arming service call timed out");
                    return true;
                }

                return false;
            },
            [this]() {
                set_destination(0, 0, 0, 0);
                RCLCPP_INFO(this->get_logger(), "Arming drone...");
            }
        };
    }

    Task NodeAPI::takeoff(float takeoff_alt) {
        return {
            [this, takeoff_alt]() ->bool {
                static bool request_sent = false;
                static int publish_count = 0;

                static auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                static rclcpp::Time start;

                req->altitude = takeoff_alt;

                if (publish_count < 100) {
                    this->local_pos_pub->publish(waypoint_g);
                    publish_count++;
                    return false;
                }

                if (!request_sent) {
                    if (!takeoff_client->service_is_ready()) {
                        return false;
                    }

                    auto future = takeoff_client->async_send_request(req);
                    request_sent = true;
                    start = this->now();
                    return false;
                }

                if (check_waypoint_reached(0.3, 999.0) == 1) {
                    RCLCPP_INFO(this->get_logger(), "Takeoff successfull, altitude %f", current_pose_g.pose.pose.position.z);
                    return true;
                }

                if ((this->now() - start).seconds() > 10.0) {
                    RCLCPP_ERROR(this->get_logger(), "Takeoff service call timed out");
                    return true;
                }

                return false;
            },
            [this, takeoff_alt]() {
                set_destination(0, 0, takeoff_alt, 0);
                RCLCPP_INFO(this->get_logger(), "Starting takeoff to altitude %f...", takeoff_alt);
            }
        };
    }

    int NodeAPI::check_waypoint_reached(float pos_tolerance, float heading_tolerance) {
        float deltaX = waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x;
        float deltaY = waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y;
        float deltaZ = waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z;
        float dMag = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);

        // RCLCPP_INFO(this->get_logger(), "dMag %f", dMag);
        // RCLCPP_INFO(this->get_logger(), "current pose x %F y %f z %f", (current_pose_g.pose.pose.position.x), (current_pose_g.pose.pose.position.y), (current_pose_g.pose.pose.position.z));
        // RCLCPP_INFO(this->get_logger(), "waypoint pose x %F y %f z %f", waypoint_g.pose.position.x, waypoint_g.pose.position.y,waypoint_g.pose.position.z);
        //check orientation

        float headingErr = fabs(fmod(current_heading_g - local_desired_heading_g + 180.0, 360.0) - 180.0);

        // RCLCPP_INFO(this->get_logger(), "current heading %f", current_heading_g);
        // RCLCPP_INFO(this->get_logger(), "local_desired_heading_g %f", local_desired_heading_g);
        // RCLCPP_INFO(this->get_logger(), "current heading error %f", headingErr);

        if( dMag < pos_tolerance && headingErr < heading_tolerance){
            return 1;
        } else{
            return 0;
        }
    }
    
    Task NodeAPI::go_to_waypoint(const WayPoint waypoint, float pos_tolerance, float heading_tolerance) {
        return {
            [this, waypoint, pos_tolerance, heading_tolerance] () -> bool {
                this->local_pos_pub->publish(waypoint_g);

                if (check_waypoint_reached(pos_tolerance, heading_tolerance) == 1) {
                    RCLCPP_INFO(this->get_logger(),
                        "Reached waypoint at (%.2f, %.2f, %.2f)",
                        waypoint.x,
                        waypoint.y,
                        waypoint.z            
                    );
                    return true;
                }

                return false;
            },
            [this, waypoint] () {
                set_destination(
                    waypoint.x,
                    waypoint.y,
                    waypoint.z,
                    waypoint.psi
                );

                RCLCPP_INFO(this->get_logger(), 
                            "Going to waypoint at (%.2f, %.2f, %.2f)",
                            waypoint.x,
                            waypoint.y,
                            waypoint.z
                );
            }
        };
    }

    Task NodeAPI::set_mode(std::string mode) {
        return {
            [this, mode]() -> bool {
                static bool request_sent = false;
                static rclcpp::Time start;
                static auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();

                req->base_mode = 0;
                req->custom_mode = mode;
                
                if (!request_sent) {
                    if (!set_mode_client->service_is_ready()) return false;

                    auto future = set_mode_client->async_send_request(req);
                    request_sent = true;
                    start = this->now();
                    return false;
                }

                if (current_state_g.mode == mode) {
                    RCLCPP_INFO(this->get_logger(), "Mode set to %s", mode.c_str());
                    return true;
                }

                if ((this->now() - start).seconds() > 10.0) {
                    RCLCPP_ERROR(this->get_logger(), "Set mode timed out");
                    return true;
                }

                return false;
            },
            [this, mode]() {
                RCLCPP_INFO(this->get_logger(), "Setting mode to %s", mode.c_str());
            }
        };
    }

    Task NodeAPI::land() {
        return {
            [this]() ->bool {
                static bool request_sent = false;
                static auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                static rclcpp::Time start;

                if (!request_sent) {
                    if (!land_client->service_is_ready()) {
                        return false;
                    }

                    auto future = land_client->async_send_request(req);
                    request_sent = true;
                    start = this->now();
                    return false;
                }

                if (!current_state_g.armed) {
                    RCLCPP_INFO(this->get_logger(), "Landing succesfull");
                    return true;
                }

                if ((this->now() - start).seconds() > 10.0) {
                    RCLCPP_ERROR(this->get_logger(), "Landing timed out");
                    return true;
                }

                return false;
            },
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Starting landing sequence...");
            }
        };
    }

    Task NodeAPI::set_speed(float speed__mps) {
        return {
            [this, speed__mps]() ->bool {
                static bool request_sent = false;
                static auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
                static rclcpp::Time start;
                static std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandLong::Response>> future;

                req->command = 178;
                req->param1 = 1;
                req->param2 = speed__mps;
                req->param3 = -1;
                req->param4 = 0;

                if (!request_sent) {
                    if (!command_client->service_is_ready()) {
                        return false;
                    }

                    future = command_client->async_send_request(req).future.share();
                    request_sent = true;
                    RCLCPP_INFO(this->get_logger(), "Speed request sent: %f m/s", speed__mps);
                    return false;
                }

                if (future.wait_for(0s) == std::future_status::ready) {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Speed accepted by FCU: %f m/s", speed__mps);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Speed request rejected by FCU");
                    }
                    request_sent = false;
                    return true;
                }

                return false;
            },
            [this, speed__mps]() {
                RCLCPP_INFO(this->get_logger(), "Setting speed to %f", speed__mps);
            }
        };
    }

    Task NodeAPI::auto_set_current_waypoint(int seq) {
        return {
            [this, seq]() -> bool {
                static bool request_sent = false;
                static auto req = std::make_shared<mavros_msgs::srv::WaypointSetCurrent::Request>();
                static std::shared_future<mavros_msgs::srv::WaypointSetCurrent::Response::SharedPtr> future;

                if (!request_sent) {
                    if (!auto_waypoint_set_current_client->service_is_ready()) {
                        return false;
                    }
                    req->wp_seq = seq;
                    future = auto_waypoint_set_current_client->async_send_request(req).future.share();
                    request_sent = true;
                    return false;
                }

                if (future.wait_for(0s) == std::future_status::ready) {
                    if (future.get()->success) {
                        RCLCPP_INFO(this->get_logger(), "Set current wp request succeeded");
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Set current wp request failed");
                    }
                    return true;
                }

                return false;
            },
            [this, seq]() {
                RCLCPP_INFO(this->get_logger(), "Setting current waypoint to #%d...", seq);
            }
        };
    }

    Task NodeAPI::set_yaw(float angle, float speed, float dir, float absolute_rel) {
        return {
            [this, angle, speed, dir, absolute_rel]() -> bool {
                static bool request_sent = false;
                static auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
                static rclcpp::Time start;
                static std::shared_future<std::shared_ptr<mavros_msgs::srv::CommandLong::Response>> future;

                req->command = 115;
                req->param1 = angle;
                req->param2 = speed;
                req->param3 = dir;
                req->param4 = absolute_rel;

                if (!request_sent) {
                    if (!command_client->service_is_ready()) {
                        return false;
                    }

                    future = command_client->async_send_request(req).future.share();
                    request_sent = true;
                    start = this->now();
                    RCLCPP_INFO(this->get_logger(), "Yaw request sent: %f [deg]", angle);
                    return false;
                }

                if (future.wait_for(0s) == std::future_status::ready) {
                    auto response = future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Yaw accepted by FCU: %f [deg]", angle);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Yaw request rejected by FCU");
                    }
                    request_sent = false;
                    return true; 
                }

                return false;
            },
            [this, angle]() {
                RCLCPP_INFO(this->get_logger(), "Setting yaw to %f degrees", angle);
            }
        };
    }

    Task NodeAPI::takeoff_global(float lat, float lon, float alt) {
        return {
            [this, lat, lon, alt]() -> bool {
                static bool request_sent = false;
                static rclcpp::Time start;
                static int publish_count = 0;
                static auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

                req->min_pitch = 0;
                req->latitude = lat;
                req->longitude = lon;
                req->altitude = alt;

                if (publish_count < 100) {
                    local_pos_pub->publish(waypoint_g);
                    publish_count++;
                    return false;
                }

                if (!request_sent) {
                    if (!takeoff_client->service_is_ready()) return false;

                    auto future = takeoff_client->async_send_request(req);
                    request_sent = true;
                    start = this->now();
                    RCLCPP_INFO(this->get_logger(), "Takeoff request sent to GPS: [%f, %f, %f]", lat, lon, alt);
                    return false;
                }

                if (current_pose_g.pose.pose.position.z >= alt * 0.95) {
                    RCLCPP_INFO(this->get_logger(), "Takeoff complete, altitude %f", current_pose_g.pose.pose.position.z);
                    return true;
                }

                if ((this->now() - start).seconds() > 15.0) {
                    RCLCPP_ERROR(this->get_logger(), "Takeoff timed out");
                    return true;
                }

                return false;
            },
            [this, lat, lon, alt]() {
                set_destination(0, 0, alt, 0);
                RCLCPP_INFO(this->get_logger(), "Initiating takeoff to GPS coordinates: [%f, %f, %f]", lat, lon, alt);
            }
        };
    }

    
}
