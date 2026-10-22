#include "SparkMax.hpp" // SPARK MAX motor controller library
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <rclcpp/rclcpp.hpp> // ROS 2 C++ library (nodes, subscribers, logging)
#include <sensor_msgs/msg/joy.hpp> // Message type published by the joystick driver

// Max drive speed in RPM (a full joystick push = this value)
const double VELOCITY_MAX = 2500.0;

// Vibrator is either fully on or off
const float VIBRATOR_OUTPUT = 1.0f;

enum CAN_IDs
{
  LEFT_MOTOR = 1,     // Left drivetrain motor
  RIGHT_MOTOR = 2,    // Right drivetrain motor
  LEFT_LIFT = 3,      // Left lift actuator
  RIGHT_LIFT = 4,     // Right lift actuator
  TILT = 5,           // Tilt actuator
  VIBRATOR = 6,       // Vibrator motor
};

// Gamepad input mapping (buttons and axes)
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
    _WINDOW_KEY = 8,
    _D_PAD_UP = 12,     // Lift Actuator UP
    _D_PAD_DOWN = 13,   // Lift Actuator DOWN
    _D_PAD_LEFT = 14,   // Tilt Actuator Up
    _D_PAD_RIGHT = 15,  // Tilt Actuator Down
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

// A ROS 2 node. It subscribes to /joy and runs joy_callback() every time a new
// controller message arrives.
class ControllerNode : public rclcpp::Node
{
public:
  // Constructor: sets up the node, motors, and the vibrator
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
    RCLCPP_INFO(this->get_logger(), "Starting controller node");
    RCLCPP_INFO(this->get_logger(), "Initializing motors and actuators");

    // Idle mode, motor type, and sensor type for each motor
    // kBrake makes a motor resist movement when it is not being driven.

    // Drivetrain settings
    leftMotor.SetIdleMode(IdleMode::kBrake);
    rightMotor.SetIdleMode(IdleMode::kBrake);
    leftMotor.SetMotorType(MotorType::kBrushless);
    rightMotor.SetMotorType(MotorType::kBrushless);
    leftMotor.SetSensorType(SensorType::kHallSensor);
    rightMotor.SetSensorType(SensorType::kHallSensor);

    // Lift actuator settings
    leftLift.SetIdleMode(IdleMode::kBrake);
    rightLift.SetIdleMode(IdleMode::kBrake);
    leftLift.SetMotorType(MotorType::kBrushed);
    rightLift.SetMotorType(MotorType::kBrushed);
    leftLift.SetSensorType(SensorType::kEncoder);
    rightLift.SetSensorType(SensorType::kEncoder);

    // Tilt actuator settings
    tilt.SetIdleMode(IdleMode::kBrake);
    tilt.SetMotorType(MotorType::kBrushed);
    tilt.SetSensorType(SensorType::kEncoder);

    // Vibrator settings
    vibrator.SetIdleMode(IdleMode::kBrake);
    vibrator.SetMotorType(MotorType::kBrushed);
    vibrator.SetSensorType(SensorType::kEncoder);

    // Invert the motors as needed
    leftMotor.SetInverted(false);
    rightMotor.SetInverted(true);
    leftLift.SetInverted(true);
    rightLift.SetInverted(true);
    tilt.SetInverted(true);
    vibrator.SetInverted(true);

    // PIDF settings for each motor PID is the feedback loop that makes a motor reach its target speed or position.
    // P reacts to current error, I to accumulated error, D to how fast the error is
    // changing, and F is a feedforward term.

    // Left drive motor
    leftMotor.SetP(0, 0.0002f);
    leftMotor.SetI(0, 0.0f);
    leftMotor.SetD(0, 0.0f);
    leftMotor.SetF(0, 0.00021f);

    // Right drive motor
    rightMotor.SetP(0, 0.0002f);
    rightMotor.SetI(0, 0.0f);
    rightMotor.SetD(0, 0.0f);
    rightMotor.SetF(0, 0.00021f);

    // Left lift
    leftLift.SetP(0, 1.51f);
    leftLift.SetI(0, 0.0f);
    leftLift.SetD(0, 0.0f);
    leftLift.SetF(0, 0.00021f);

    // Right lift
    rightLift.SetP(0, 1.51f);
    rightLift.SetI(0, 0.0f);
    rightLift.SetD(0, 0.0f);
    rightLift.SetF(0, 0.00021f);

    // Tilt
    tilt.SetP(0, 1.51f);
    tilt.SetI(0, 0.0f);
    tilt.SetD(0, 0.0f);
    tilt.SetF(0, 0.00021f);

    // Save all of the settings above to each motor controller's flash memory
    leftMotor.BurnFlash();
    rightMotor.BurnFlash();
    leftLift.BurnFlash();
    rightLift.BurnFlash();
    tilt.BurnFlash();
    vibrator.BurnFlash();

    RCLCPP_INFO(this->get_logger(), "Eureka! Motor Controllers Initialized");

    // Subscribe to the /joy topic to receive joystick input
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Eureka! Controller node is ready.");

    // Send a first heartbeat to every motor controller. SPARK MAX controllers need
    // regular heartbeats to stay enabled. The try/catch keeps the node from crashing
    // if one of them fails.
    try
    {
      leftMotor.heartBeat();
      rightMotor.heartBeat();
      leftLift.heartBeat();
      rightLift.heartBeat();
      tilt.heartBeat();
      vibrator.heartBeat();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "heartbeat no more sad: %s", e.what());
    }
  }

private:
  // SPARK MAX motor controllers for the drivetrain, lift, tilt, and vibrator
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMax leftLift;
  SparkMax rightLift;
  SparkMax tilt;
  SparkMax vibrator;

  // Tracks whether the vibrator is on, and the previous button state for edge detection
  bool vibrator_active;
  bool prev_vibrator_button = false;

  // Handle for the /joy subscription
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // Runs once for every message received from /joy
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Send a heartbeat to every motor controller so they stay enabled
    leftMotor.heartBeat();
    rightMotor.heartBeat();
    leftLift.heartBeat();
    rightLift.heartBeat();
    tilt.heartBeat();
    vibrator.heartBeat();

    // Tank drive: left stick controls the left side, right stick controls the right side
    float left_y = msg->axes[Gp::_LEFT_VERTICAL_STICK];
    float right_y = msg->axes[Gp::_RIGHT_VERTICAL_STICK];

    // Clamp each value to the range -1.0 to 1.0
    float left_drive_raw = std::max(-1.0f, std::min(1.0f, left_y));
    float right_drive_raw = std::max(-1.0f, std::min(1.0f, right_y));

    // Convert stick position (-1 to 1) to a target speed in RPM
    double left_speed = left_drive_raw * VELOCITY_MAX;
    double right_speed = right_drive_raw * VELOCITY_MAX;

    // Send the target speeds to the drive motors over CAN
    leftMotor.SetVelocity(left_speed);
    rightMotor.SetVelocity(right_speed);

    // Vibrator toggle (right bumper)
    bool current_vibrator = msg->buttons[Gp::_RIGHT_BUMPER];

    // Only toggle on the moment the button goes from not pressed to pressed,
    // so holding the button doesn't flip the vibrator on every message
    if (current_vibrator && !prev_vibrator_button)
    {
      vibrator_active = !vibrator_active;
      vibrator.SetDutyCycle(vibrator_active ? VIBRATOR_OUTPUT : 0.0f);
      RCLCPP_INFO(this->get_logger(), "Vibrator toggled %s", vibrator_active ? "ON" : "OFF");
    }

    // Remember this button state for the next message
    prev_vibrator_button = current_vibrator;

    // Print drive speeds (RPM) and vibrator state, at most once per second
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Left Motor: %.2f | Right Motor: %.2f | Vibrator: %s",
        left_speed, right_speed, vibrator_active ? "ON" : "OFF");
  }
};

int main(int argc, char *argv[])
{
  // Start ROS 2
  rclcpp::init(argc, argv);

  // Name of the CAN interface the motor controllers are on
  std::string can_interface = "can0";

  // Create the node and spin it so it keeps receiving joystick messages
  // until the program is stopped
  auto node = std::make_shared<ControllerNode>(can_interface);
  rclcpp::spin(node);

  // Shut ROS 2 down cleanly
  rclcpp::shutdown();
  return 0;
}