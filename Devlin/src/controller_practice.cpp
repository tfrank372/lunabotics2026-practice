#include "SparkMax.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>


const float VIBRATOR_OUTPUT = 1.0f;
const float ACTUATOR_OUTPUT = 1.0f;

// Incase of any stickdrift or if the controller is off by a bit bet someone
// is trying to rest the stick, it will still be 0
const float JOYSTICK_DEADBAND = 0.05f;

enum CAN_IDs
{
    LEFT_MOTOR = 1,
    RIGHT_MOTOR = 2,
    LEFT_LIFT = 3,
    RIGHT_LIFT = 4,
    TILT = 5,
    VIBRATOR = 6,
};

namespace Gp
{
    enum Buttons
    {
        _A = 0,            // free
        _B = 1,            // free
        _X = 2,            // free
        _Y = 3,            // free
        _LEFT_BUMPER = 4,  // free
        _RIGHT_BUMPER = 5, // Vibration toggle
        _D_PAD_UP = 12,    // Lift up
        _D_PAD_DOWN = 13,  // Lift down
        _D_PAD_LEFT = 14,  // Tilt up
        _D_PAD_RIGHT = 15, // Tilt down
    };

    enum Axes
    {
        _LEFT_HORIZONTAL_STICK = 0,
        _LEFT_VERTICAL_STICK = 1,
        _RIGHT_HORIZONTAL_STICK = 2,
        _RIGHT_VERTICAL_STICK = 3,
    };
}

class ControllerNode : public rclcpp::Node
{
public:
    explicit ControllerNode(const std::string &can_interface)
        : Node("controller_node"),
          leftMotor(can_interface, LEFT_MOTOR),
          rightMotor(can_interface, RIGHT_MOTOR),
          leftLift(can_interface, LEFT_LIFT),
          rightLift(can_interface, RIGHT_LIFT),
          tilt(can_interface, TILT),
          vibrator(can_interface, VIBRATOR),
          vibrator_active(false)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing motor controllers");

        leftMotor.SetIdleMode(IdleMode::kBrake);
        rightMotor.SetIdleMode(IdleMode::kBrake);
        leftMotor.SetMotorType(MotorType::kBrushless);
        rightMotor.SetMotorType(MotorType::kBrushless);
        leftMotor.SetSensorType(SensorType::kHallSensor);
        rightMotor.SetSensorType(SensorType::kHallSensor);

        leftLift.SetIdleMode(IdleMode::kBrake);
        rightLift.SetIdleMode(IdleMode::kBrake);
        tilt.SetIdleMode(IdleMode::kBrake);
        vibrator.SetIdleMode(IdleMode::kBrake);

        leftLift.SetMotorType(MotorType::kBrushed);
        rightLift.SetMotorType(MotorType::kBrushed);
        tilt.SetMotorType(MotorType::kBrushed);
        vibrator.SetMotorType(MotorType::kBrushed);

        leftLift.SetSensorType(SensorType::kEncoder);
        rightLift.SetSensorType(SensorType::kEncoder);
        tilt.SetSensorType(SensorType::kEncoder);
        vibrator.SetSensorType(SensorType::kEncoder);

        leftMotor.SetInverted(false);
        rightMotor.SetInverted(true);
        leftLift.SetInverted(true);
        rightLift.SetInverted(true);
        tilt.SetInverted(true);
        vibrator.SetInverted(true);

        leftMotor.SetP(0, 0.0002f);
        leftMotor.SetI(0, 0.0f);
        leftMotor.SetD(0, 0.0f);
        leftMotor.SetF(0, 0.00021f);

        rightMotor.SetP(0, 0.0002f);
        rightMotor.SetI(0, 0.0f);
        rightMotor.SetD(0, 0.0f);
        rightMotor.SetF(0, 0.00021f);

        leftLift.SetP(0, 1.51f);
        leftLift.SetI(0, 0.0f);
        leftLift.SetD(0, 0.0f);
        leftLift.SetF(0, 0.00021f);

        rightLift.SetP(0, 1.51f);
        rightLift.SetI(0, 0.0f);
        rightLift.SetD(0, 0.0f);
        rightLift.SetF(0, 0.00021f);

        tilt.SetP(0, 1.51f);
        tilt.SetI(0, 0.0f);
        tilt.SetD(0, 0.0f);
        tilt.SetF(0, 0.00021f);

        leftMotor.BurnFlash();
        rightMotor.BurnFlash();
        leftLift.BurnFlash();
        rightLift.BurnFlash();
        tilt.BurnFlash();
        vibrator.BurnFlash();

        RCLCPP_INFO(this->get_logger(), "Motor controllers initialized");

        // Subscribe to joy. (importnant)
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));

        // Timer that sends a heartbeat do they actually get the joy input
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            [this]()
            {
                try
                {
                    SparkBase::Heartbeat();
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Exception in heartbeat: %s", e.what());
                }
            });

        RCLCPP_INFO(this->get_logger(), "Controller node ready");
    }

private:
    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkMax leftLift;
    SparkMax rightLift;
    SparkMax tilt;
    SparkMax vibrator;

    bool vibrator_active;                
    bool prev_vibrator_button = false;   

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    static float apply_deadband(float value)
    {
        float clamped = std::max(-1.0f, std::min(1.0f, value));
        return (std::fabs(clamped) < JOYSTICK_DEADBAND) ? 0.0f : clamped;
    }

    static float button_pair_output(bool positive, bool negative)
    {
        return (static_cast<float>(positive) - static_cast<float>(negative)) * ACTUATOR_OUTPUT;
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() <= Gp::_RIGHT_VERTICAL_STICK ||
            msg->buttons.size() <= Gp::_D_PAD_RIGHT)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Joy message has too few axes/buttons (%zu axes, %zu buttons)",
                                 msg->axes.size(), msg->buttons.size());
            return;
        }

        float left_drive = apply_deadband(msg->axes[Gp::_LEFT_VERTICAL_STICK]);
        float right_drive = apply_deadband(msg->axes[Gp::_RIGHT_VERTICAL_STICK]);.
        float left_speed = left_drive * VELOCITY_MAX;
        float right_speed = right_drive * VELOCITY_MAX;

        leftMotor.SetVelocity(left_speed);
        rightMotor.SetVelocity(right_speed);

        // Vibrator toggle
        bool current_vibrator = msg->buttons[Gp::_RIGHT_BUMPER] != 0;
        if (current_vibrator && !prev_vibrator_button)
        {
            vibrator_active = !vibrator_active;
            vibrator.SetDutyCycle(vibrator_active ? VIBRATOR_OUTPUT : 0.0f);
            RCLCPP_INFO(this->get_logger(), "Vibrator toggled %s", vibrator_active ? "ON" : "OFF");
        }
        prev_vibrator_button = current_vibrator;

        float lift_output = button_pair_output(msg->buttons[Gp::_D_PAD_UP] != 0,
                                               msg->buttons[Gp::_D_PAD_DOWN] != 0);
        leftLift.SetDutyCycle(lift_output);
        rightLift.SetDutyCycle(lift_output);

        tilt.SetDutyCycle(button_pair_output(msg->buttons[Gp::_D_PAD_LEFT] != 0,
                                             msg->buttons[Gp::_D_PAD_RIGHT] != 0));

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Left: %.0f RPM | Right: %.0f RPM | Lift: %.1f | Vibrator: %s",
            left_speed, right_speed, lift_output, vibrator_active ? "ON" : "OFF");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::string can_interface = "can0";

    // spin() keeps the node alive, processing joystick messages and timer
    // ticks, until you shut it down (Ctrl+C).
    auto node = std::make_shared<ControllerNode>(can_interface);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
