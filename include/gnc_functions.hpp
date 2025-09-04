#pragma once

#include <unistd.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/position_target.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/srv/waypoint_set_current.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_pull.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

struct gnc_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	float psi; ///< rotation about the third axis of your reference frame
};

class GNCFunctions : public rclcpp::Node, public std::enable_shared_from_this<GNCFunctions> {
    public:
        GNCFunctions() : GNCFunctions("gnc_node") {}

        GNCFunctions(const std::string& node_name) : Node(node_name) {
            init_publishers_subscribers();
        }
    
    protected:
        void init_publishers_subscribers();

        void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
            current_state_g = *msg;
        }

        geometry_msgs::msg::Point enu_2_local(nav_msgs::msg::Odometry current_pose_enu);
        void pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
        geometry_msgs::msg::Point get_current_location();
     
        float get_current_heading()
        {
	        return current_heading_g;
        }
        
        void set_heading(float heading);
        void set_destination(float x, float y, float z, float psi);
        void set_destination_lla(float lat, float lon, float alt, float heading);
        void set_destination_lla_raw(float lat, float lon, float alt);
        int wait4connect();
        int wait4start();
        int initialize_local_frame();
        int arm();
        int takeoff(float takeoff_alt);
        int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01);
        int set_mode(std::string mode);
        int land();
        int set_speed(float speed__mps);
        int auto_set_current_waypoint(int seq);
        int set_yaw(float angle, float speed, float dir, float absolute_rel);
        int takeoff_global(float lat, float lon, float alt);
        
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub; 
        rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr global_lla_pos_pub;
        rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr global_lla_pos_pub_raw;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr currentPos;
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
        rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client;
        rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr command_client;
        rclcpp::Client<mavros_msgs::srv::WaypointPull>::SharedPtr auto_waypoint_pull_client;
        rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr auto_waypoint_push_client;
        rclcpp::Client<mavros_msgs::srv::WaypointSetCurrent>::SharedPtr auto_waypoint_set_current_client;
        
        mavros_msgs::msg::State current_state_g;
        nav_msgs::msg::Odometry current_pose_g;
        geometry_msgs::msg::Pose correction_vector_g;
        geometry_msgs::msg::Point local_offset_pose_g;
        geometry_msgs::msg::PoseStamped waypoint_g;

        float current_heading_g;
        float local_offset_g;
        float correction_heading_g = 0;
        float local_desired_heading_g; 
};
