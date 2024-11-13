#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "std_srvs/srv/set_bool.hpp"

namespace husky_highlevel_controller
{

    class Subscriber : public rclcpp::Node
    {
    public:
        Subscriber()
            : Node("distance_stop", "husky_highlevel_controller")
        {
            rclcpp::QoS qos(rclcpp::KeepLast(10));
            subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "/a200_0000/sensors/lidar2d_0/scan",
                qos,
                std::bind(&Subscriber::callback, this, std::placeholders::_1));

            client_ = create_client<std_srvs::srv::SetBool>("toggle_subscription");
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
        bool stopped_ = false;

    private:
        void callback(const sensor_msgs::msg::LaserScan &msg)
        {
            auto min = std::min_element(msg.ranges.begin(), msg.ranges.end());
            auto distance = *min;
            // emergency stop if distance is less than 0.5m
            if (distance < 0.5 && !stopped_)
            {
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = false;
                auto result = client_->async_send_request(request);
                RCLCPP_INFO_STREAM(get_logger(), "Emergency stop");
                stopped_ = true;
            } else if(distance > 0.5 && stopped_) {
                stopped_ = false;
            }
            
        }
    };
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<husky_highlevel_controller::Subscriber>());
    rclcpp::shutdown();
    return 0;
}
