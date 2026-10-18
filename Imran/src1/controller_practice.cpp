#include "SparkMax.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

// Practice: write a ROS 2 node that reads the Xbox controller (/joy topic)
// and drives the robot's motors through the SPARK MAX controllers over CAN.
// max motor speed in RPM (joystick value of 1.0 = full speed)
const double VELOCITY_MAX = 2500.0;

const float VIBRATOR_OUTPUT = 1.0f;

// CAN IDs for each motor controller on the robot (from the README)
enum CAN_IDs
{
    LEFT_MOTOR = 1,
    RIGHT_MOTOR = 2,
    LEFT_LIFT = 3,
    RIGHT_LIFT = 4,
    TILT = 5,
    VIBRATOR = 6,
};

// Gamepad mapping (from the README)
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

class ControllerNode : public rclcpp::Node
{
  public:
    explicit ControllerNode(const std::string &can_interface)
    : Node("imran_controller_node"),
    //for 61-66 each one of those lines calls a specific motor controller
    //(LEFT_MOTOR(1), RIGHT_MOTOR(2), LEFT_LIFT(3), etc. and set themselves up in the can bus (can_interface)
      leftMotor(can_interface, LEFT_MOTOR), 
      rightMotor(can_interface, RIGHT_MOTOR),
      leftLift(can_interface, LEFT_LIFT),
      rightLift(can_interface, RIGHT_LIFT),
      Tilt(can_inteface, Tilt),
      vibrator(can_inteface, vibrator)
      vibrator_active(false) //false so when the rover gets turned on the vibrator doesnt start as well

    { 
      RCLCPP_INFO(this->get_logger(), "Begin Initializing Node");
      RCLCPP_INFO(this->get_logger(), "Initializing Motor Controllers");
      //[INFO] [1760700000.123456789] [imran_controller_node]: Initializing Motor Controllers
      
      leftMotor.SetIdleMode(IdleMode::kBrake); //what happens when you send 0 power
      //kbrake - the controller shorts the motor's windings which make the motor
      //resist turning. The robot stops quickly and stays put.
      rightMotor.SetIdleMode(IdleMode::kBrake); // same thing
      leftMotor.SetMotorType(MotorType::kBrushless); //what kind of motor is plugged in
      //brushed motors are simple - apply voltage, and they spin.
      //brushless motors need the contoller to switch power between three wires in the right order,
      //timed to where the rotor is.
      rightMotor.SetMotorType(MotorType::kBrushless);
      leftMotor.SetSensorType(SensorType::kHallSensor);
      rightMotor.SetSensorType(SensorType::kHallSensor);
      
      leftLift.SetIdleMode(IdleMode::kBrake);
      rightLift.SetIdleMode(IdleMode::kBrake);
      leftLift.SetMotorType(MotorType::kBrushed);
      rightLift.SetMotorType(MotorType::kBrushed);
      leftLift.SetSensorType(SensorType::kEncoder);
      rightLift.SetSensorType(SensorType::kEncoder);
      //initalizes the setting for the lift actuators

      tilt.SetIdleMode(IdleMode::kBrake);
      tilt.SetMotorType(MotorType::kBrushed);
      tilt.SensorType(SensorType::kEncoder);

      vibrator.SetIdlemode(IdleMode::kBrake);
      vibrator.SetMotorType(MotorType::kBrushed);
      vibrator.SetSensorType(SensorType::kEncoder);

      leftMotor.SetInverted(false);
      rightMotor.SetInverted(true);
      leftLift.SetInverted(true);
      rightLift.SetInverted(true);
      vibrator.SetInverted(true);
      tilt.SetInverted(true);
      
      //pid settings for left motor
      leftMotor.SetP(0,0.0002f);
      leftMotor.SetI(0,0.0f);
      leftMotor.SetD(0,0.0f);
      leftMotor.SetF(0,0.00021f);

      //pid settings for right motor
      rightMotor.SetP(0,0.0002f);
      rightMotor.SetI(0,0.0f);
      rightMotor.SetD(0,0.0f);
      rightMotor.SetF(0,0.00021f);

      //pid settings for left lift
      leftLift.SetP(0, 1.51f);
      leftLift.SetI(0, 0.0f);
      leftLift.SetD(0, 0.0f);
      leftLift.SetF(0, 0.00021f);

      //pid settings for left lift
      rightLift.SetP(0, 1.51f);
      rightLift.SetI(0, 0.0f);
      rightLift.SetD(0, 0.0f);
      rightLift.SetF(0, 0.00021f);

      // PID settings for tilt
        
      leftMotor.BurnFlash();
      rightMotor.BurnFlash();
      leftLift.BurnFlash();
      rightLift.BurnFlash();
      tilt.BurnFlash();
      vibrator.BurnFlash();
      //save settings for each one of them

      RCLCPP_INFO(this->get_logger(), "Motor Controllers Initialized");
      //so many libraries that im not aware of wtf
      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",10,
        std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1)
      );
      RCLCPP_INFO(this->get_logger(), "we got joystick");
      //Send initial heartbears (catch exceptions if thrown)

      try {
        leftMotor.heartBeat();
        rightMotor.heartBeat();
        leftLift.heartBeat();
        rightLift.heartBeat();
        tilt.heartBeat();
        vibrator.heartBeat();
      }

      catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in heartbeat: %s", e.what());
      }


    
private:
  //we don't want any code outside this class accessing these variables
  //one sparkmax object for each motor controller
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMax leftLift;
  SparkMax rightLift;
  SparkMax tilt;
  SparkMax vibrator;

  //bool = think boolean variable(true/flase)
  bool vibrator_active = false;  // vibrator_active stores the state 
  //there was a bug because value wasnt given for a variable, in cpp that
  //may lead to variable storing leftover garbage from the memory
  bool prev_vibrator_button = false; //the callback runs many times
  //per second. holding the button for half a second to press the button
  //might mean 10 messages saying "pressed." Without memory, the vibrator 
  //would flip on and off 10 times. by comparing now with the last time the code
  //can detect the single moment the press starts

  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  //from ros2 cpp library -> a listener for a topic (Subscription) -> that listens
  //for "Joy" <> fills in which message type, held through a shared ptr

    
  //we want to void the function because we don't want it to return anything back to ROS
  //the function just acts on any input and doesn't need to return any values
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {

    //sends heartbeats to motors so they are acctually getting the joy input
    leftMotor.heartBeat();
    rightMotor.heartBeat();
    vibrator.heartBeat();
    rightLift.heartBeat();
    leftLift.heartBeat();
    tilt.heartBeat();
    
    //get the vertical axis from each joystick stick
    //we are setting up for tank drive so each joystick only moves one side of the robot
    float left_y = msg->axes[Gp::_LEFT_VERTICAL_STICK];   // Left joystick controls left side
    float right_y = msg->axes[Gp::_RIGHT_VERTICAL_STICK]; // Right joystick controls right side

    //makes it so the robot isn't going at 100% the whole time. gives the joystick a range of values 
    float left_drive_raw = std::max(-1.0f, std::min(1.0f, left_y));
    float right_drive_raw = std::max(-1.0f, std::min(1.0f, right_y));
    
    //compute motor target velocities
    //this is a double and not a float because doubles are more accurate with more digits of precision 
    double left_speed = left_drive_raw * VELOCITY_MAX;
    double right_speed = right_drive_raw * VELOCITY_MAX;

    //send the velocity commands to the motors over CAN
    leftMotor.SetVelocity(left_speed);
    rightMotor.SetVelocity(right_speed);

    //vibrator toggle (using right bumper button)
    //continuously checks if the right bumped is being pressed, and if it is pressed it updates the variable and toggles the vibrator
    bool current_vibrator = (msg->buttons[Gp::_RIGHT_BUMPER]);
    if (current_vibrator && !prev_vibrator_button)
    {
      vibrator_active = !vibrator_active;
      vibrator.SetDutyCycle(vibrator_active ? VIBRATOR_OUTPUT : 0.0f);
      RCLCPP_INFO(this->get_logger(), "Vibrator toggled %s", vibrator_active ? "ON" : "OFF");
    }

    //this will ensure that the vibrator isn't continuously activated again and again more like a toggle
    prev_vibrator_button = current_vibrator;

    //will continuously update in the terminal every second with the % of each motor and if the vibrator is on
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Left Motor: %.2f | Right Motor: %.2f | Vibrator: %s",
        left_speed, right_speed, vibrator_active ? "ON" : "OFF");
  }
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);


  std::string can_interface = "can0";

  // Create and spin the node
  //we want to spin the node so that it continues to recieve inputs until we turn it off
  auto node = std::make_shared<ControllerNode>(can_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}












