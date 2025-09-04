#include "gnc_functions.hpp"
#include <rclcpp/rclcpp.hpp>
//include API 

using namespace std::chrono_literals;

class GNCExample : public GNCFunctions {
	public:
		GNCExample() : GNCFunctions("gnc_example") {
			auto timer = this->create_wall_timer(
				500ms, [this]() {
					this->run();
				}
			);
		}

		void run() {
			wait4connect();
			wait4start();
			initialize_local_frame();
			arm();
			takeoff(3.0);
			rclcpp::sleep_for(std::chrono::seconds(10));
			land();
		}
	
	private:
		rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto gnc_node = std::make_shared<GNCExample>();
    rclcpp::spin(gnc_node);
    rclcpp::shutdown();
    return 0;
}
