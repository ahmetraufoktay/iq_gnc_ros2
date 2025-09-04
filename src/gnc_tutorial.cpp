#include "gnc_functions.hpp"
#include <rclcpp/rclcpp.hpp>
//include API 

using namespace std::chrono_literals;

class GNCExample : public GNCFunctions {
	public:
		GNCExample() : GNCFunctions("gnc_example") {
			wait4connect();
			wait4start();
			initialize_local_frame();
		}

		void run() {
			arm();
			set_mode("GUIDED");
			takeoff(3.0);
			rclcpp::sleep_for(std::chrono::seconds(10));
			land();
		}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto gnc_node = std::make_shared<GNCExample>();
    rclcpp::spin_some(gnc_node);
	gnc_node->run();
    rclcpp::shutdown();
    return 0;
}
