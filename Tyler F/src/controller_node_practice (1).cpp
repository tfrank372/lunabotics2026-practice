#include "SparkMax.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>

const float VELOCITY_MAX = 2500.0;  // rpm, after gearbox turns into 11.1 RPM

enum CAN_IDs
{
  LEFT_MOTOR = 1,
  RIGHT_MOTOR = 2,
};

namespace Gp
{
  enum Buttons
  {
    _A = 0,             // Excavation Autonomy
    _B = 1,             // Stop Automation
    _X = 2,             // Excavation Reset
    _Y = 3,             // Deposit Autonomy
    _LEFT_BUMPER = 4,   // Alternate between control modes
    _RIGHT_BUMPER = 5,  // Vibration Toggle
    _LEFT_TRIGGER = 6,  // Safety Trigger
    _RIGHT_TRIGGER = 7, // Safety Trigger
    _WINDOW_KEY = 8,    // Button 8 /** I do not know what else to call this key */
    _D_PAD_UP = 12,     // Lift Actuator UP
    _D_PAD_DOWN = 13,   // Lift Actuator DOWN
    _D_PAD_LEFT = 14,   // Tilt Actuator Up   /** CHECK THESE */
    _D_PAD_RIGHT = 15,  // Tilt Actuator Down /** CHECK THESE */
    _X_BOX_KEY = 16
  };

  enum Axes
  {
    _LEFT_HORIZONTAL_STICK = 0,
    _LEFT_VERTICAL_STICK = 1,
    _RIGHT_HORIZONTAL_STICK = 2,
    _RIGHT_VERTICAL_STICK = 3,
  };
}

// HEARTBEAT
try
{
    leftMotor.Heartbeat();
    rightMotor.Heartbeat();
}


catch(const std::exception& e)
{
    std::cerr << e.what() << '\n';
}


class ControllerNode : public rclcpp::Node
{
  public:

  ControllerNode(const std::string &can_interface)
  : Node("controller_node"),
    leftMotor(can_interface, LEFT_MOTOR),
    rightMotor(can_interface, RIGHT_MOTOR)
    {
      RCLCPP_INFO(this.get_logger(), "beginnign node...")
      RCLCPP_INFO(this.get_logger(), "configuring motor controllers..")

      leftMotor.SetIdleMode(IdleMode::kBrake);
      rightMotor.SetIdleMode(IdleMode::kBrake);
      leftMotor.SetMotorType(MotorType::kBrushless);
      rightMotor.SetMotorType(MotorType::kBrushless);
      // finish configuring
      // leftMotor.Burnflash()
      // rightMotor.BurnFlash()


    }
    //rostopic
    // /joy
    // /controller_node
    {
      joy_subscriber_ = this.create_subscription<general_msgs::msg::Joy>(
        "/joy", 10,
        std::bind::(&ControllerNode::joy_callback, this, std::placeholders::_1));
        RCLCP_INFO(this.get_logger(), "we got joy!");

      
    }
    // request 

    {
      void send_excavation_request()
      {
        if (!excavation_client_)
          {
            RCLCP_INFO(this.get_logger(), "cant find excvation");
            return;
          }
        auto request = std::make_shared<controller_pkg::srv::ExcavationRequest::Request>();
        request.start_excavation = true;
        RCLCP_INFO(this.get_logger(), "we got excavation");

      }
    }
    
    //ADDED FUNCTION THAT BINDS VALUE AND COMPUTES STEP OUTPUT
    float joystickToDutyCycle(float value) {
        float uBound = 1.0f;
        float lBound = -1.0f;
        
        if (value > uBound) {
            value = uBound;
        }
        else if (value < lBound) {
            value = lBound;
        }

        float absValue = std:fabs(value);

        if (absValue < 0.25f) {
            return 0.0f;
        }
        else if (absValue < 0.5f) {
            return (value > 0 ? 0.25f : -0.25f);
        }
        else if (absValue < 0.75f) {
            return (value > 0 ? 0.5f : -0.5f);
        }
        else if (absValue < 1.0f) {
            return (value > 0 ? 0.75f : -0.75f);
        }

        return (value > 0 ? 1.0f : -1.0f);  
    }

    // drivetrain
    {
      float left_drive = 0.0f;
      float right_drive = 0.0f;

      float leftJS = -joy_msg.axes[Gp::Axes::_LEFT_VERTICAL_STICK];
      float rightJS = -joy_msg.axes[Gp::Axes::_RIGHT_VERTICAL_STICK];

      left_drive = joystickToDutyCycle(leftJS);
      right_drive = joystickToDutyCycle(rightJS);

      leftMotor.SetDutyCycle(left_drive);
      rightMotor.SetDutyCycle(right_drive);
      leftMotor.HeartBeat();
      rightMotor.HeartBeat();
    }



}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::string can_interface = "can0";
  auto temp_node = rclcpp::Node::make_shared("controller_param_node");
  temp_node->declare_parameter<std::string>("can_interface", "can0");
  temp_node->get_parameter("can_interface", can_interface);
  auto node = std::make_shared<ControllerNode>(can_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
