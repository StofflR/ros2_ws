#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "std_srvs/srv/set_bool.hpp"

namespace husky_highlevel_controller
{

    class Subscriber : public rclcpp::Node
    {
    public:
        Subscriber()
            : Node("crash_stop", "husky_highlevel_controller")
        {
            rclcpp::QoS qos(rclcpp::KeepLast(10));
            subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
                "/a200_0000/sensors/imu_0/data",
                qos,
                std::bind(&Subscriber::callback, this, std::placeholders::_1));

            client_ = create_client<std_srvs::srv::SetBool>("toggle_subscription");
        }

    private:
        // subscribe to imu data
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
        bool stopped_ = false;

    private:
        void callback(const sensor_msgs::msg::Imu &msg)
        {
            // detect crash by checking acceleration
            auto acc = msg.linear_acceleration;
            
            // check if the acceleration is greater than 10m/s^2
            auto acc_z  = acc.z;
            bool crash = acc_z > 9.9;

            // emergency stop 
            if (crash && !stopped_)
            {
                RCLCPP_INFO_STREAM(get_logger(), "Total acceleration: " << acc_z);
                auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                request->data = false;
                auto result = client_->async_send_request(request);
                RCLCPP_INFO_STREAM(get_logger(), "Emergency stop");
            } else if(!crash && stopped_) {
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
