#include <rclcpp/rclcpp.hpp>
#include "gnc_node.cpp"
#include "yolov8_msgs/msg/yolov8_inference.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <vector>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class SRNode : public GNC::NodeAPI {
    public: 
        SRNode() : NodeAPI("sr_node"), mode_g(0), person_found(false)
        {
            yolo_sub = this->create_subscription<yolov8_msgs::msg::Yolov8Inference>(
                "/Yolov8_Inference", 
                rclcpp::QoS(10), 
                [this](const yolov8_msgs::msg::Yolov8Inference::SharedPtr msg){this->yolo_callback(msg);}
            );

            img_sub = this->create_subscription<sensor_msgs::msg::Image>(
                "/cam/image_raw",
                rclcpp::QoS(10),
                [this](const sensor_msgs::msg::Image::SharedPtr msg){this->img_callback(msg);}
            );

            img_pub = this->create_publisher<sensor_msgs::msg::Image>("/inference_result_cv2", rclcpp::QoS(10));

            apply_plan();
        }
    
    private:
        void apply_plan() {
            mission_queue.push(wait4connect());
            mission_queue.push(wait4start());
            mission_queue.push(initialize_local_frame());
			mission_queue.push(arm());
            mission_queue.push(takeoff(10.0));

            float spacing = 10;
            float range = 50;
            int rows = 5;
            int row; 
            
            for (int i = 0; i < rows; ++i) {
                row = i * 2;
                wp.x = row * spacing; wp.y = 0; wp.z = 10; wp.psi = 0;
                mission_queue.push(SearchAndRescue(wp, 0.3, 10));

                wp.x = row * spacing; wp.y = range; wp.z = 10; wp.psi = 0;
                mission_queue.push(SearchAndRescue(wp, 0.3, 10));

                wp.x = (row + 1) * spacing; wp.y = range; wp.z = 10; wp.psi = 0;
                mission_queue.push(SearchAndRescue(wp, 0.3, 10));


                wp.x = (row + 1) * spacing; wp.y = 0; wp.z = 10; wp.psi = 0;
                mission_queue.push(SearchAndRescue(wp, 0.3, 10));
            }

            mission_queue.push(land());
        }

        GNC::Task SearchAndRescue(GNC::WayPoint wp, float pos_tolerance = 0.3, float heading_tolerance = 5) {
            return {
                [this, wp, pos_tolerance, heading_tolerance]() -> bool {
                    this->local_pos_pub->publish(waypoint_g);
                    
                    if (this->mode_g == 1) {
                        if (!this->person_found) {
                            RCLCPP_INFO(this->get_logger(), "PERSON FOUND! Rescue mission is starting...");
                            this->person_found = true;
                        }
                        return true;
                    } else if (check_waypoint_reached(pos_tolerance, heading_tolerance) == 1) {
                        RCLCPP_INFO(this->get_logger(), "Search WP at (%f, %f, %f) reached.", wp.x, wp.y, wp.z);
                        return true;
                    } else {
                        return false;
                    }
                },
                [this, wp]() {
                    if (this->mode_g != 1) {
                        RCLCPP_INFO(this->get_logger(), "Set course to waypoint (%f, %f, %f)", wp.x, wp.y , wp.z);
                        set_destination(
                            wp.x,
                            wp.y,
                            wp.z,
                            wp.psi
                        );
                    }
                }
            };
        }

        void yolo_callback(const yolov8_msgs::msg::Yolov8Inference::SharedPtr msg) {
            static int cnt = 0;
            
            for (const auto &r : msg->yolov8_inference) {
                std::string class_name = r.class_name;
                int top = r.top;
                int left = r.left;
                int right = r.right;
                int bottom = r.bottom;

                RCLCPP_INFO(this->get_logger(), "%d %s : %d, %d, %d, %d",
                    cnt, class_name.c_str(), top, left, bottom, right);
                
                cv::rectangle(img_, cv::Point(top, left), cv::Point(bottom, right),
                      cv::Scalar(255, 255, 0), 2);
                cnt++;

                if (mode_g != 1 && r.class_name == "person") {
                    mode_g = 1;
                }
            }

            cnt = 0;

            cv_bridge::CvImage cv_img;
            cv_img.header = msg->header;
            cv_img.encoding = "bgr8";
            cv_img.image = img_;

            img_pub->publish(*cv_img.toImageMsg());
        }

        void img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                this->img_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            } catch (cv_bridge::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception %s", e.what());
            }
        }

        rclcpp::Subscription<yolov8_msgs::msg::Yolov8Inference>::SharedPtr yolo_sub; 
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;
        
        rclcpp::TimerBase::SharedPtr timer_;
        
        cv::Mat img_;
        GNC::WayPoint wp;
        int mode_g;
        bool person_found;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SRNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
