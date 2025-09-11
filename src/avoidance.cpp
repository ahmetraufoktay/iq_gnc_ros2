#include <rclcpp/rclcpp.hpp>
#include "gnc_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>
#include <mutex>

class AvoidanceNode : public GNC::NodeAPI
{
    public:
        AvoidanceNode() 
        : NodeAPI("avoidance_node") 
        {
            laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/iris/laser_scan",
                rclcpp::QoS(10),
                [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){this->scan_cb(msg);}
            );

            apply_plan();
        }
    
    private:
        void apply_plan() {
            mission_queue.push(wait4connect());
            mission_queue.push(wait4start());
            mission_queue.push(initialize_local_frame());
            mission_queue.push(arm());
            mission_queue.push(takeoff(2.0));
        }


        void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            float avoidance_vector_x = 0; 
            float avoidance_vector_y = 0;
            bool avoid = false;
            
            for(size_t i=1; i < msg->ranges.size(); ++i)
            {
                float d0 = 3; 
                float k = 0.5;

                if(msg->ranges[i] < d0 && msg->ranges[i] > .35)
                {
                    avoid = true;
                    float x = cos(msg->angle_increment*i);
                    float y = sin(msg->angle_increment*i);
                    float U = -.5*k*pow(((1/msg->ranges[i]) - (1/d0)), 2);	

                    avoidance_vector_x = avoidance_vector_x + x*U;
                    avoidance_vector_y = avoidance_vector_y + y*U;

                }
            }
            float current_heading = get_current_heading();
            float deg2rad = (M_PI/180);
            avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
            avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);

            if(avoid)
            {
                if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 3)
                {
                    avoidance_vector_x = 3 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
                    avoidance_vector_y = 3 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
                }
                geometry_msgs::msg::Point current_pos = get_current_location();
                set_destination(avoidance_vector_x + current_pos.x, avoidance_vector_y + current_pos.y, 2, 0);	
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto avoidance = std::make_shared<AvoidanceNode>();
    rclcpp::spin(avoidance);
    rclcpp::shutdown();
    return 0;
}
