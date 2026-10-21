#include"SparkMax.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <algorithm>
const double VELOCITY_MAX=2500.0;
 enum CAN_IDs{ LEFT_MOTOR = 1, 
RIGHT_MOTOR = 2, 
LEFT_LIFT = 3, 
RIGHT_LIFT = 4, 
TILT = 5, 
VIBRATOR = 6 };
namespace Gp { enum Buttons { _A = 0,
_B = 1, 
_X = 2,
_Y = 3,
_LEFT_BUMPER = 4, 
_RIGHT_BUMPER = 5,
_LEFT_TRIGGER = 6, 
_RIGHT_TRIGGER = 7, 
_WINDOW_KEY = 8,
_D_PAD_UP = 12, 
_D_PAD_DOWN = 13, 
_D_PAD_LEFT = 14, 
_D_PAD_RIGHT = 15, 
_X_BOX_KEY = 16 }
};
class ControllerNode : public rclcpp::Node
{
public:
    explicit ControllerNode(const std::string &can_interface)
    : Node("practice_node"),
      leftMotor(can_interface, LEFT_MOTOR),
      rightMotor(can_interface, RIGHT_MOTOR)
    {
               leftMotor.SetIdleMode(IdleMode::kBrake);
        rightMotor.SetIdleMode(IdleMode::kBrake);
        leftMotor.SetMotorType(MotorType::kBrushless);
        rightMotor.SetMotorType(MotorType::kBrushless);
        rightMotor.SetInverted(true);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1)
        );
    }
}
private:
    SparkMax leftMotor;
    SparkMax rightMotor;
       void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
        // --- Left Motor ---
        float left_y = msg->axes[Gp::_LEFT_VERTICAL_STICK];
        float left_speed = std::max(-1.0f, std::min(1.0f, left_y)) * VELOCITY_MAX;
        leftMotor.SetVelocity(left_speed);

        // --- Right Motor (PUT THIS HERE) ---
        float right_y = msg->axes[Gp::_RIGHT_VERTICAL_STICK];
        float right_speed = std::max(-1.0f, std::min(1.0f, right_y)) * VELOCITY_MAX;
        rightMotor.SetVelocity(right_speed);
    }
}
};
int main(int argc,char*argv[]){
auto node=std::make_shared<ControllerNode>("can0");
  rlcpp::spin(node);
  rclpp::shutdown();
  return 0;
}
