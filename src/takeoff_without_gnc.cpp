#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class DroneController : public rclcpp::Node
{
  public:
    DroneController() :
      Node("drone_controller")
    {
      state_sub = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state",
        rclcpp::QoS(10),
        [this](const mavros_msgs::msg::State::SharedPtr msg){
          this->state_callback(msg);
        }
      );

      pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose",
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);}
      );

      arming_client = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
      set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
      takeoff_client = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

      pose_goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/local",
        rclcpp::QoS(10)
      );
    }

    void run() {
      arm();
      set_mode("GUIDED");
      takeoff(3.0);
      rclcpp::sleep_for(std::chrono::seconds(10));
      land();
    }

  private:
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal_pub;
  
    mavros_msgs::msg::State current_state;
    geometry_msgs::msg::PoseStamped current_pose;

    
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
      current_state = *msg;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      current_pose = *msg;
    }

    int arm() {
      while (!arming_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      set_mode("GUIDED");
      auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      req->value = true;

      auto future = arming_client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
          == rclcpp::FutureReturnCode::SUCCESS) 
      {
        RCLCPP_INFO(this->get_logger(), "Arming Succesfull %d", future.get()->success);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Arming Failed");
      }

      return 0;
    }

    int set_mode(std::string mode) {
      while (!set_mode_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
      req->base_mode = 0;
      req->custom_mode = mode;

      auto future = set_mode_client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
        == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Set Mode to %s. Mode Sent : %d", mode.c_str(), future.get()->mode_sent);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Set Mode Failed : %d", future.get()->mode_sent);
      }
      
      return 0;
    }

    int takeoff(float target_altitude) {
      while (!takeoff_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          return -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
      req->altitude = target_altitude;

      auto future = takeoff_client->async_send_request(req);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future)
        == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Takeoff succesfull. CODE: %d", future.get()->success);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Takeoff failed. CODE: %d", future.get()->success);
      }

      return 0;
    }

    int land() {
      set_mode("LAND");
      RCLCPP_INFO(this->get_logger(), "Landing...");
      
      return 0;
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto drone = std::make_shared<DroneController>();
  rclcpp::spin_some(drone);
  drone->run();
  rclcpp::shutdown();
  return 0;
}
