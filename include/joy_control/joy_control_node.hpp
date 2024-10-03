#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace custom_joy
{

  // XBox One constants
  constexpr int JOY_AXIS_LEFT_LR = 0;       // Left stick left/right; 1.0 is left and -1.0 is right
  constexpr int JOY_AXIS_LEFT_FB = 1;       // Left stick forward/back; 1.0 is forward and -1.0 is back
  constexpr int JOY_AXIS_RIGHT_LR = 3;      // Right stick left/right; 1.0 is left and -1.0 is right
  constexpr int JOY_AXIS_RIGHT_FB = 4;      // Right stick forward/back; 1.0 is forward and -1.0 is back

  constexpr int JOY_BUTTON_VIEW = 4;        // View button
  constexpr int JOY_BUTTON_MENU = 5;        // Menu button
  constexpr int JOY_BUTTON_X = 0;        // X button
  constexpr int JOY_BUTTON_CIRCLE = 1;        // Circle button
  constexpr int JOY_BUTTON_SQUARE = 2;        // Square button
  constexpr int JOY_BUTTON_TRIANGLE = 3;        // Triangle button
  constexpr int JOY_BUTTON_UP = 11;     //Up button
  constexpr int JOY_BUTTON_DOWN = 12;     //Down button
  constexpr int JOY_BUTTON_LEFT = 6;     //Left button
  constexpr int JOY_BUTTON_RIGHT = 7;      //Right button



  class CustomJoyNode : public rclcpp::Node
  {
  public:

    explicit CustomJoyNode(const rclcpp::NodeOptions &options);

    ~CustomJoyNode();

  private:

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    //Services for 4 main buttons
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr x_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr circle_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr rectangle_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr triangle_client_;

    //Services for 4 direction buttons
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr b_up_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr b_down_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr b_left_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr b_right_client_;

    // XBox One assignments
    const int joy_axis_throttle_ = JOY_AXIS_RIGHT_FB;
    const int joy_axis_strafe_ = JOY_AXIS_RIGHT_LR;
    const int joy_axis_vertical_ = JOY_AXIS_LEFT_FB;
    const int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
    const int joy_button_takeoff_ = JOY_BUTTON_MENU;
    const int joy_button_land_ = JOY_BUTTON_VIEW;

    const int joy_button_x_ = JOY_BUTTON_X;
    const int joy_button_circle_ = JOY_BUTTON_CIRCLE;
    const int joy_button_rectangle_ = JOY_BUTTON_SQUARE;
    const int joy_button_triangle_ = JOY_BUTTON_TRIANGLE;

    const int joy_button_up_= JOY_BUTTON_UP;
    const int joy_button_down_= JOY_BUTTON_DOWN;
    const int joy_button_left_= JOY_BUTTON_LEFT;
    const int joy_button_right_= JOY_BUTTON_RIGHT;


  };

} // namespace custom_joy
