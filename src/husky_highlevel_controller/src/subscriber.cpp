#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace husky_highlevel_controller
{

  class Subscriber : public rclcpp::Node
  {
  public:
    Subscriber()
        : Node("subscriber", "husky_highlevel_controller")
    {
      rcl_interfaces::msg::ParameterDescriptor topic_param_desc;
      topic_param_desc.description = "Topic name to subscribe to";

      rcl_interfaces::msg::ParameterDescriptor queue_param_desc;
      queue_param_desc.description = "Queue size to use for the subscription";

      declare_parameter("topic", "/a200_0000/sensors/lidar2d_0/scan", topic_param_desc);
      declare_parameter("queue_size", 10, queue_param_desc);

      param_callback_handel_ = add_on_set_parameters_callback(
        std::bind(&Subscriber::handleOnSetParameter, this, std::placeholders::_1)
      );
      
      rclcpp::QoS qos(rclcpp::KeepLast(get_parameter("queue_size").as_int()));
      subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
          get_parameter("topic").as_string(),
          qos,
          std::bind(&Subscriber::callback, this, std::placeholders::_1));
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handel_;

  private:
    void callback(const sensor_msgs::msg::LaserScan &msg)
    {
      auto min = std::min_element(msg.ranges.begin(), msg.ranges.end());
      RCLCPP_INFO_STREAM(get_logger(), "Minimum range: " << *min);
    }

    rcl_interfaces::msg::SetParametersResult handleOnSetParameter(
        const std::vector<rclcpp::Parameter> &params)
    {
      size_t queue_size = get_parameter("queue_size").as_int();
      auto topic = get_parameter("topic").as_string();

      for (rclcpp::Parameter param : params)
      {
        rclcpp::Parameter current_param = get_parameter(param.get_name());

        if (param != current_param)
        {
          RCLCPP_INFO_STREAM(
              get_logger(),
              "Parameter: " << param.get_name() << " is about to change from \""
                            << current_param.value_to_string() << "\" to \""
                            << param.value_to_string() << "\"");
          if (param.get_name() == "queue_size")
            queue_size = param.as_int();
          
          if (param.get_name() == "topic")
            topic = param.as_string();
        }
      }
      RCLCPP_INFO_STREAM(get_logger(), "Creating new subscriber with topic: " << topic << " and queue size: " << queue_size);
      
      auto new_subscriber = create_subscription<sensor_msgs::msg::LaserScan>(
          topic,
          rclcpp::QoS(rclcpp::KeepLast(queue_size)),
          std::bind(&Subscriber::callback, this, std::placeholders::_1));
      subscriber_.swap(new_subscriber);
      
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      return result;
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
