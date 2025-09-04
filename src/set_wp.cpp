#include "gnc_functions.hpp"
#include <rclcpp/rclcpp.hpp>
//include API 

class SetWPNode : public GNCFunctions {
	public:
		SetWPNode() : GNCFunctions() {
			wait4connect();
            wait4start();
            arm();
            takeoff_global(30.2, 80.5, 5);
            

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                [this]() {this->control_loop();}
            );
        }
	
	private:
        void control_loop() {
            // Set yaw (angle, speed, direction, relative)
            set_yaw(10, 20, -1, 1);

            // Set global LLA destination
            set_destination_lla_raw(-35.364261, 149.165230, 5);
        }

        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto wp_node = std::make_shared<SetWPNode>();
    rclcpp::spin(wp_node);
    rclcpp::shutdown();
    return 0;
}
