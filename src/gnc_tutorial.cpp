#include "gnc_functions.hpp"
#include <rclcpp/rclcpp.hpp>
//include API 

class GNCExample : public GNCFunctions {
	public:
		GNCExample() : GNCFunctions() {
			wait4connect();
			wait4start();
			initialize_local_frame();
			arm();
		}

	private:
    	std::vector<gnc_api_waypoint> waypointList;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto gnc_node = std::make_shared<GNCExample>();
    rclcpp::spin(gnc_node);
    rclcpp::shutdown();
    return 0;
}
