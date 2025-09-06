#include "gnc_node.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class GNCExample : public GNC::NodeAPI {
	public:
		GNCExample() : NodeAPI("gnc_example") {
			mission_queue.push(wait4connect());
			mission_queue.push(wait4start());
			mission_queue.push(initialize_local_frame());
			mission_queue.push(arm());
			mission_queue.push(takeoff(3.0));
		}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GNCExample>());
    rclcpp::shutdown();
    return 0;
}
