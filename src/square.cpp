#include "gnc_node.hpp"
#include <rclcpp/rclcpp.hpp>
//include API 

class SquareNode : public GNC::NodeAPI {
	public:
		SquareNode()
		 : NodeAPI("square_node")
		{
			mission_queue.push(wait4connect());
			mission_queue.push(wait4start());
			mission_queue.push(initialize_local_frame());
			mission_queue.push(arm());
			mission_queue.push(takeoff(3.0));

			nextWayPoint.x = 5.0; nextWayPoint.y = 0.0; nextWayPoint.z = 3.0; nextWayPoint.psi = -90.0;
			mission_queue.push(go_to_waypoint(nextWayPoint));

			nextWayPoint.x = 5.0; nextWayPoint.y = 5.0; nextWayPoint.z = 3.0; nextWayPoint.psi = 0.0;
			mission_queue.push(go_to_waypoint(nextWayPoint));

			nextWayPoint.x = 0.0; nextWayPoint.y = 5.0; nextWayPoint.z = 3.0; nextWayPoint.psi = 90.0;
			mission_queue.push(go_to_waypoint(nextWayPoint));
			
			nextWayPoint.x = 0.0; nextWayPoint.y = 0.0; nextWayPoint.z = 3.0; nextWayPoint.psi = 180.0;
			mission_queue.push(go_to_waypoint(nextWayPoint));

			mission_queue.push(land());
		}
	
		private:
			GNC::WayPoint nextWayPoint;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto square_node = std::make_shared<SquareNode>();
    rclcpp::spin(square_node);
    rclcpp::shutdown();
    return 0;
}
