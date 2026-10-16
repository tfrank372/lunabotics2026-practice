#include "SparkMax.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

const double max_speed = 2500.0;
const float duty_cycle = 1.0f;

const float vibrator_output = 1.0f;

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
        _A = 0,  // Vibration Toggle
    }

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
        explicit ControllerNode(const std::string &can_interface) : 
        Node("chris_crazy_crontroller"), 
        leftMotor(can_interface, LEFT_MOTOR), 
        rightMotor(can_interface, RIGHT_MOTOR),
        leftLift(can_interface, LEFT_LIFT),
        rightLift(can_interface, RIGHT_LIFT),
        tilt(can_interface, TILT),
        vibrator(can_interface, VIBRATOR),
        vibrator_on(false)
        {
            //setting idle mods, sensor type, and motor type for each motor; This just acts with aakash's sparkmax library
            leftMotor.SetIdleMode(IdleMode::kBrake);
            rightMotor.SetIdleMode(IdleMode::kBrake);
            leftMotor.SetMotorType(MotorType::kBrushless);
            rightMotor.SetMotorType(MotorType::kBrushless);
            leftMotor.SetSensorType(SensorType::kHallSensor);
            rightMotor.SetSensorType(SensorType::kHallSensor);
            
            //sets dirction for each motor, and vibrator t/f
            leftMotor.SetInverted(false);
            rightMotor.SetInverted(true);
            vibrator.SetInverted(true);

            //sets PID for left motor
            leftMotor.SetP(0, 0.0002f);
            leftMotor.SetI(0, 0.0f);
            leftMotor.SetD(0, 0.0f);
            leftMotor.SetF(0, 0.00021f);

            //sets PID for right motor
            rightMotor.SetP(0, 0.0002f);
            rightMotor.SetI(0, 0.0f);
            rightMotor.SetD(0, 0.0f);
            rightMotor.SetF(0, 0.00021f);

            //this one says burn flash
            leftMotor.BurnFlash();
            rightMotor.BurnFlash();
            vibrator.BurnFlash();

            RCLCPP_INFO(this->get_logger(), "The PID stuff and the sparkmax stuff has been successfully done");



        //this is the constructor of the object
        RCLCPP_INFO(this->get_logger(), "actuators, motors, and vibrator intialized!");

        //subscribe to joy node
        joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "happy as a clam");


        //will try to get a heartbeat over and over until it works
        try{
            leftMotor.heartBeat();
            rightMotor.heartBeat();
            vibrator.heartBeat();
        } catch(const std::exception &error_exception)
        RCLCPP_ERROR(this->get_logger(), "Exception in heartbeat: %s" error_exception.what());
        RCLCPP_INFO(this->get_logger(), "heartbeat beating");
        }

    private:
        SparkMax rightMotor;
        SparkMax tilt;
        SparkMax vibrator;

        bool vibrations;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            //constantly publishes a heartbeat to make sure nothing broken
            leftMotor.heartBeat();
            rightMotor.heartBeat();
            vibrator.heartBeat();
            RCLCPP_ERROR(this->get_logger(), "heartbeat failed");

            //deciding left wheel speed
            float left_joy_raw = msg->axes[Gp::_LEFT_VERTICAL_STICK];
            float left_drive = std::max(1.0f, std::min(-1.0f, left_joy_raw));
            float left_speed = left_joy_raw * max_speed;

            //deciding right wheel speed
            float right_joy_raw = msg->Axes[Gp::_RIGHT_VERTICAL_STICK];
            float right_drive = std::max(1.0f, std::min(-1.0f, right_joy_raw));
            float right_speed = right_joy_raw * max_speed;

            //setting motors to determined wheel speed
            left_motor.setVelocity(left_speed);
            right_motor.setVelocity(right_speed);

            //turns vibrator on or off
            float A_button_pressed = msg->Buttons[Gp::_A];

            if (A_button_pressed)
            {
            vibrator_on = !vibrator_on;
            vibrator.SetDutyCycle(vibrator_on ? vibrator_output : 0.0f);
            RCLCPP_INFO(this->get_logger(), vibrator_on ? "Vibrator ON" : "Vibrator OFF");
            };

            RCLCPP_INFO_THROTTLE(this->get_logger(); this->get_clock())
            
        }
};



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);


  std::string can_interface = "can0";

  //spin makes it keep on recieving input until you turn it off (keeps on executing the node)
  auto node = std::make_shared<ControllerNode>(can_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
};