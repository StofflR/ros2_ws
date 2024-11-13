#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_srvs/srv/set_bool.hpp"

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
          std::bind(&Subscriber::handleOnSetParameter, this, std::placeholders::_1));

      rclcpp::QoS qos(rclcpp::KeepLast(get_parameter("queue_size").as_int()));
      subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
          get_parameter("topic").as_string(),
          qos,
          std::bind(&Subscriber::callback, this, std::placeholders::_1));

      publisher_ = create_publisher<geometry_msgs::msg::Twist>("/a200_0000/cmd_vel", 10);
      publisher_estimate_ = create_publisher<visualization_msgs::msg::Marker>("/estimate", 10);
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      service_ = create_service<std_srvs::srv::SetBool>(
          "toggle_subscription",
          std::bind(&Subscriber::toggle_subscription, this, std::placeholders::_1, std::placeholders::_2));
    }
  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_estimate_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handel_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    tf2_ros::Buffer::SharedPtr tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool is_subscribed_ = true;

  private:
    void toggle_subscription(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
      if (request->data)
      {
        is_subscribed_ = true;
        response->message = "Resumed subscription";
      }
      else
      {
        is_subscribed_ = false;
        response->message = "Paused subscription";
      }
      response->success = true;
    }    

    void callback(const sensor_msgs::msg::LaserScan &msg)
    {
      auto min = std::min_element(msg.ranges.begin(), msg.ranges.end());
      auto min_angle = msg.angle_min + std::distance(msg.ranges.begin(), min) * msg.angle_increment;
      auto distance = *min;
      // turn robot to angle and move in direction of the minimum range
      geometry_msgs::msg::Twist twist;
      // check if the minimum distance is less than 0.5m and not infinity
      if(is_subscribed_)
      {
        if (distance > 0.0 && distance != std::numeric_limits<float>::infinity())
          twist.linear.x = distance;
        twist.angular.z = min_angle;
      }
      publisher_->publish(twist);
      geometry_msgs::msg::Point trans;

      try
      {
        std::string source = "lidar2d_0_laser";
        std::string target = "odom";
        if (tf_buffer_->canTransform(target, source, tf2::TimePointZero))
        {
          auto transform = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
          geometry_msgs::msg::Point point;
          point.x = distance * cos(min_angle);
          point.y = distance * sin(min_angle);
          tf2::doTransform(point, trans, transform);
        }
      }
      catch (const tf2::TransformException &e)
      {
        RCLCPP_ERROR_STREAM(get_logger(), e.what());
        return;
      }

      // for odom baselink
      visualization_msgs::msg::Marker estimate_laser;
      estimate_laser.header.frame_id = "odom";
      estimate_laser.header.stamp = now();
      estimate_laser.id = 0;
      estimate_laser.type = visualization_msgs::msg::Marker::CYLINDER;
      estimate_laser.action = visualization_msgs::msg::Marker::MODIFY;
      estimate_laser.pose.position = trans;
      estimate_laser.scale.x = 0.5;
      estimate_laser.scale.y = 0.5;
      estimate_laser.scale.z = 0.5;
      estimate_laser.color.a = 1.0;
      estimate_laser.color.r = 0.0;
      estimate_laser.color.g = 1.0;
      estimate_laser.color.b = 0.0;
      publisher_estimate_->publish(estimate_laser);

      RCLCPP_INFO_STREAM(get_logger(), "Minimum range: " << distance << " at angle: " << min_angle);
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
