#include "gnc_node.hpp"
#include <rclcpp/rclcpp.hpp>

class SetWPNode : public GNC::NodeAPI {
	public:
		SetWPNode() : NodeAPI("set_wp") {
			mission_queue.push(wait4connect());
			mission_queue.push(wait4start());
			mission_queue.push(initialize_local_frame());
			mission_queue.push(arm());
            mission_queue.push(takeoff_global(30.2, 80.5, 5));

            mission_queue.push(set_yaw(10, 20, -1, 1));
            mission_queue.push({
                [this]() -> bool {
                    set_destination_lla_raw(-35.364261, 149.165230, 5);
                    return true; 
                },
                []() {
                    RCLCPP_INFO(rclcpp::get_logger("set_wp"), "Publishing raw LLA waypoint");
                }
            });          
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto wp_node = std::make_shared<SetWPNode>();
    rclcpp::spin(wp_node);
    rclcpp::shutdown();
    return 0;
}
