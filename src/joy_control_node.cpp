#include "joy_control/joy_control_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace custom_joy
{

  CustomJoyNode::CustomJoyNode(const rclcpp::NodeOptions &options) :
    Node("custom_joy", options)
  {
    using std::placeholders::_1;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&CustomJoyNode::joy_callback, this, _1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_joy", 1);

    x_client_ = create_client<std_srvs::srv::Trigger>("x");
    circle_client_ = create_client<std_srvs::srv::Trigger>("circle");
    rectangle_client_ = create_client<std_srvs::srv::Trigger>("rectangle");
    triangle_client_ = create_client<std_srvs::srv::Trigger>("triangle");

    b_up_client_ = create_client<std_srvs::srv::Trigger>("b_up");
    b_down_client_ = create_client<std_srvs::srv::Trigger>("b_down");
    b_left_client_ = create_client<std_srvs::srv::Trigger>("b_left");
    b_right_client_ = create_client<std_srvs::srv::Trigger>("b_right");

    (void) joy_sub_;
  }



  CustomJoyNode::~CustomJoyNode()
  {}


  void CustomJoyNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {

    if (joy_msg->buttons[joy_button_x_]) {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      x_client_->async_send_request(request);

    } else if (joy_msg->buttons[joy_button_circle_]) {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      circle_client_->async_send_request(request);

    } else if (joy_msg->buttons[joy_button_rectangle_]) {   
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      rectangle_client_->async_send_request(request);

    } else if (joy_msg->buttons[joy_button_triangle_]) { 
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      triangle_client_->async_send_request(request);

    } else if (joy_msg -> buttons[joy_button_up_]){
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      b_up_client_->async_send_request(request);

    } else if (joy_msg -> buttons[joy_button_down_]){
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      b_down_client_->async_send_request(request);

    } else if (joy_msg -> buttons[joy_button_left_]){
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      b_left_client_->async_send_request(request);

    } else if (joy_msg -> buttons[joy_button_right_]){
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      b_right_client_->async_send_request(request);
      
    }else {
      geometry_msgs::msg::Twist twist_msg;
      twist_msg.linear.x = joy_msg->axes[joy_axis_throttle_];
      twist_msg.linear.y = joy_msg->axes[joy_axis_strafe_];
      twist_msg.linear.z = joy_msg->axes[joy_axis_vertical_];
      twist_msg.angular.z = joy_msg->axes[joy_axis_yaw_];
      cmd_vel_pub_->publish(twist_msg);
    }
  }

} // namespace tello_joy

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(custom_joy::CustomJoyNode)