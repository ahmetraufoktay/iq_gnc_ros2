#include "gnc_functions.hpp"
#include <rclcpp/rclcpp.hpp>
//include API 

class SquareNode : public GNCFunctions {
	public:
		SquareNode() : GNCFunctions() {
			gnc_api_waypoint nextWayPoint;

			nextWayPoint.x = 0; nextWayPoint.y = 0; nextWayPoint.z = 3; nextWayPoint.psi = 0;
			waypointList.push_back(nextWayPoint);

			nextWayPoint.x = 5; nextWayPoint.y = 0; nextWayPoint.z = 3; nextWayPoint.psi = -90;
			waypointList.push_back(nextWayPoint);

			nextWayPoint.x = 5; nextWayPoint.y = 5; nextWayPoint.z = 3; nextWayPoint.psi = 0;
			waypointList.push_back(nextWayPoint);

			nextWayPoint.x = 0; nextWayPoint.y = 5; nextWayPoint.z = 3; nextWayPoint.psi = 90;
			waypointList.push_back(nextWayPoint);

			nextWayPoint.x = 0; nextWayPoint.y = 0; nextWayPoint.z = 3; nextWayPoint.psi = 180;
			waypointList.push_back(nextWayPoint);

			nextWayPoint.x = 0; nextWayPoint.y = 0; nextWayPoint.z = 3; nextWayPoint.psi = 0;
			waypointList.push_back(nextWayPoint);

			wait4connect();
			wait4start();
			initialize_local_frame();
			arm();
			takeoff(3.0);
		}

		void run() {
			rclcpp::Rate rate(2.0); // 2 Hz control loop
			size_t counter = 0;

			while (rclcpp::ok())
			{
				rclcpp::spin_some(this->get_node_base_interface()); // process subscriptions/callbacks

				if (check_waypoint_reached(0.3) == 1)
				{
					if (counter < waypointList.size())
					{
						set_destination(
							waypointList[counter].x,
							waypointList[counter].y,
							waypointList[counter].z,
							waypointList[counter].psi
						);
						counter++;
					}
					else
					{
						// Land after all waypoints are reached
						land();
						break; // exit loop after landing
					}
				}

				rate.sleep();
			}
		}
	
	private:
    	std::vector<gnc_api_waypoint> waypointList;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto square_node = std::make_shared<SquareNode>();
    square_node->run();
    rclcpp::shutdown();
    return 0;
}
