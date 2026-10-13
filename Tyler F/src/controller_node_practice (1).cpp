#include "SparkMax.hpp"
#include <cmath>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

//notes: caps are used for values that are constant/don't change
//rclcpp is the ros library for cpp and how we are creating nodes and subscribers 

//is double because it is a precise value
//2500.0 is the max motor speed in rpm
//if a joystick returns 1.0 for example that is the max and it would cause the rpm to spin at 2500
const double VELOCITY_MAX = 2500.0; 

//we want the vibrator either fully on or fully off so it can be set at either 1 or 0
const float VIBRATOR_OUTPUT = 1.0f;

//with this we are grabbing constants for these values to uniquely identify each component
enum CAN_IDs
{
    LEFT_MOTOR = 1,
    RIGHT_MOTOR = 2,
    LEFT_LIFT = 3, 
    RIGHT_LIFT = 4,
    TILT = 5,
    VIBRATOR = 6,
};


//this is giving us the controller input mapping so that we can identify each button pressed on the controller
//gp = gamepad input
//we want to define a namespace so that the variables don't become confusing and are grouped accordingly
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

//defining a controller node with access to ROS features
class ControllerNode : public rclcpp::Node
{
    //set to public so that anything included in here can be accessed outside of the class
    public:
        //this is a constructor and it is used to set up our variables before we actually use them later on
        //the constructor is automatically called and we don't have to call it later on
        explicit ControllerNode(const std::string &can_interface)
        : Node("controller_node"),
          //each line calls a specific motor/object and sets it up in the CAN bus
          //this is basically assembling each individual part
          leftMotor(can_interface, LEFT_MOTOR),
          rightMotor(can_interface, RIGHT_MOTOR),
          leftLift(can_interface, LEFT_LIFT),
          rightLift(can_interface, RIGHT_LIFT),
          tilt(can_interface, TILT),
          vibrator(can_interface, VIBRATOR),
          //we want this false so that the vibrator isn't already running when we start the robot
          vibrator_active(false)
          {
            RCLCPP_INFO(this->get_logger(), "Begin Initializing Node");
        
            RCLCPP_INFO(this->get_logger(), "Initializing Motor Controllers");
        
            leftMotor.SetIdleMode(IdleMode::kBrake);
            rightMotor.SetIdleMode(IdleMode::kBrake);
            leftMotor.SetMotorType(MotorType::kBrushless);
            rightMotor.SetMotorType(MotorType::kBrushless);
            leftMotor.SetSensorType(SensorType::kHallSensor);
            rightMotor.SetSensorType(SensorType::kHallSensor);
            // Initializes the settings for the drivetrain motors
        
            leftLift.SetIdleMode(IdleMode::kBrake);
            rightLift.SetIdleMode(IdleMode::kBrake);
            leftLift.SetMotorType(MotorType::kBrushed);
            rightLift.SetMotorType(MotorType::kBrushed);
            leftLift.SetSensorType(SensorType::kEncoder);
            rightLift.SetSensorType(SensorType::kEncoder);
            // Initializes the settings for the lift actuators
        
            tilt.SetIdleMode(IdleMode::kBrake);
            tilt.SetMotorType(MotorType::kBrushed);
            tilt.SetSensorType(SensorType::kEncoder);
            // Initializes the settings for the tilt actuator
        
            vibrator.SetIdleMode(IdleMode::kBrake);
            vibrator.SetMotorType(MotorType::kBrushed);
            vibrator.SetSensorType(SensorType::kEncoder);
            // Initializes the settings fro the vibrator
        
            leftMotor.SetInverted(false);
            rightMotor.SetInverted(true);
            leftLift.SetInverted(true);
            rightLift.SetInverted(true);
            tilt.SetInverted(true);
            vibrator.SetInverted(true);
            // Initializes the inverting status
        
            leftMotor.SetP(0, 0.0002f);
            leftMotor.SetI(0, 0.0f);
            leftMotor.SetD(0, 0.0f);
            leftMotor.SetF(0, 0.00021f);
            // PID settings for left motor
        
            rightMotor.SetP(0, 0.0002f);
            rightMotor.SetI(0, 0.0f);
            rightMotor.SetD(0, 0.0f);
            rightMotor.SetF(0, 0.00021f);
            // PID settings for right motor
        
            leftLift.SetP(0, 1.51f);
            leftLift.SetI(0, 0.0f);
            leftLift.SetD(0, 0.0f);
            leftLift.SetF(0, 0.00021f);
            // PID settings for left lift
        
            rightLift.SetP(0, 1.51f);
            rightLift.SetI(0, 0.0f);
            rightLift.SetD(0, 0.0f);
            rightLift.SetF(0, 0.00021f);
            // PID settings for right lift
        
            // PID settings for tilt
            tilt.SetP(0, 1.51f);
            tilt.SetI(0, 0.0f);
            tilt.SetD(0, 0.0f);
            tilt.SetF(0, 0.00021f);
            // PID settings for tilt
        
            leftMotor.BurnFlash();
            rightMotor.BurnFlash();
            leftLift.BurnFlash();
            rightLift.BurnFlash();
            tilt.BurnFlash();
            vibrator.BurnFlash();
            RCLCPP_INFO(this->get_logger(), "Motor Controllers Initialized");

            //now we need to subscribe to joy so that we can recieve inputs from the contorller joystick
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
              "/joy", 10,
              std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "we got joy");
            // Send initial heartbeats (catch exceptions if thrown)
            try {
              leftMotor.heartBeat();
              rightMotor.heartBeat();
              leftLift.heartBeat();
              rightLift.heartBeat();
              tilt.heartBeat();
              vibrator.heartBeat();
            } catch (const std::exception &e) {
              RCLCPP_ERROR(this->get_logger(), "Exception in heartbeat: %s", e.what());
            }
        }


private:
  //we don't want any code outside this class accessing these variables
  //adding each varible to sparkmax
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMax leftLift;
  SparkMax rightLift;
  SparkMax tilt;
  SparkMax vibrator;

  //bool = think boolean variable(true/flase)
  bool vibrator_active;  // toggles vibrator motor on/off
  bool prev_vibrator_button = false; //need this for edge protection

  
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;


    
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











