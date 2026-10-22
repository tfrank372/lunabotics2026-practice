/**
 *
 * New features on top of the original node:
 *   1. SAFETY
 *        - Deadman trigger: the robot only moves while the trigger is held
 *        - B button = STOP: stops everything and stays stopped until you
 *          release and re-squeeze the deadman trigger
 *        - Watchdog: if controller input stops for 0.5 s, everything stops
 *        - Crash protection: errors stop the motors instead of crashing mid-drive
 *        - Motors are stopped when the node shuts down (Ctrl+C)
 *   2. LIFT on D-pad up/down, TILT on D-pad left/right
 *   3. SLOW / FAST drive modes on the left bumper (starts in SLOW)
 *   4. Ramp rates and current limits on the motor controllers
 *
 *
 */

#include "SparkMax.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;

// =====================================================================
//  TUNING CONSTANTS - change these, rebuild, and test
// =====================================================================

// ---- Drive ----
constexpr double VELOCITY_MAX       = 2500.0; // motor RPM at full stick in FAST mode
constexpr double SLOW_MODE_SCALE    = 0.30;   // SLOW mode = 30% of VELOCITY_MAX
constexpr bool   START_IN_SLOW_MODE = true;   // safer to start slow

// ---- Actuators (duty cycle: 0.0 = off, 1.0 = full power) ----
constexpr float LIFT_SPEED      = 0.5f;  // lift power while D-pad up/down is held
constexpr float TILT_SPEED      = 0.4f;  // tilt power while D-pad left/right is held
constexpr float VIBRATOR_OUTPUT = 1.0f;  // vibrator power when ON

// ---- Ramp rates (seconds to go from 0 to full output) ----
constexpr float DRIVE_RAMP_RATE    = 0.5f;   // drive motors (closed-loop velocity)
constexpr float ACTUATOR_RAMP_RATE = 0.25f;  // lift + tilt (open-loop duty cycle)

// ---- Current limits (amps) ----
constexpr uint16_t DRIVE_CURRENT_LIMIT    = 40;
constexpr uint16_t ACTUATOR_CURRENT_LIMIT = 20; 

// ---- Safety timing ----
constexpr auto JOY_TIMEOUT      = 500ms;  // stop everything if no /joy message this long
constexpr auto HEARTBEAT_PERIOD = 20ms;   // how often heartbeat + watchdog run
constexpr bool BURN_FLASH_ON_STARTUP = true;  // save config to controller memory at startup

// =====================================================================
//  HARDWARE + CONTROLLER MAPPING
// =====================================================================

enum CAN_IDs
{
  LEFT_MOTOR  = 1,
  RIGHT_MOTOR = 2,
  LEFT_LIFT   = 3,
  RIGHT_LIFT  = 4,
  TILT        = 5,
  VIBRATOR    = 6,
};

namespace Gp
{
  enum Buttons
  {
    _A = 0,
    _B = 1,              // STOP (latched)
    _X = 2,
    _Y = 3,
    _LEFT_BUMPER = 4,    // SLOW / FAST mode toggle
    _RIGHT_BUMPER = 5,   // Vibrator toggle
    _LEFT_TRIGGER = 6,   // Deadman (hold to drive)
    _RIGHT_TRIGGER = 7,
    _WINDOW_KEY = 8,
    _D_PAD_UP = 12,      // Lift UP
    _D_PAD_DOWN = 13,    // Lift DOWN
    _D_PAD_LEFT = 14,    // Tilt UP
    _D_PAD_RIGHT = 15,   // Tilt DOWN
    _X_BOX_KEY = 16
  };

  enum Axes
  {
    _LEFT_HORIZONTAL_STICK = 0,
    _LEFT_VERTICAL_STICK = 1,
    _RIGHT_HORIZONTAL_STICK = 2,
    _RIGHT_VERTICAL_STICK = 3,   // CHECK: on many Xbox controllers this is axis 4
  };
}

// ---- Input options: confirm these with `ros2 topic echo /joy` ----
// Some controllers report triggers and the D-pad as AXES instead of buttons.
constexpr int   DEADMAN_BUTTON        = Gp::_LEFT_TRIGGER;
constexpr bool  DEADMAN_IS_AXIS       = false; // true if pulling the trigger moves an axis
constexpr int   DEADMAN_AXIS          = 2;     // only used when DEADMAN_IS_AXIS is true
constexpr float TRIGGER_PRESSED_BELOW = -0.5f; // trigger axes rest at +1.0, fully pulled = -1.0

constexpr bool DPAD_IS_AXES         = false;  // true if the D-pad moves axes
constexpr int  DPAD_HORIZONTAL_AXIS = 6;      // left = +1.0, right = -1.0
constexpr int  DPAD_VERTICAL_AXIS   = 7;      // up   = +1.0, down  = -1.0

using JoyMsg = sensor_msgs::msg::Joy;

// =====================================================================
//  CONTROLLER NODE
// =====================================================================

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
    vibrator(can_interface, VIBRATOR)
  {
    RCLCPP_INFO(get_logger(), "Initializing motor controllers...");
    configure_motors();
    stop_all();  // make sure nothing is moving before we start listening
    RCLCPP_INFO(get_logger(), "Motor controllers initialized");

    joy_sub_ = create_subscription<JoyMsg>(
      "/joy", 10, std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));

    // The safety timer sends heartbeats and runs the watchdog
    safety_timer_ = create_wall_timer(
      HEARTBEAT_PERIOD, std::bind(&ControllerNode::safety_timer_callback, this));

    RCLCPP_INFO(get_logger(),
      "Ready. Robot is DISABLED until you hold the deadman trigger. Drive mode: %s",
      slow_mode_ ? "SLOW" : "FAST");
  }

  // Runs when the node is destroyed (for example after Ctrl+C)
  ~ControllerNode() override
  {
    stop_all();
  }

private:
  // ---------------- Hardware ----------------
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMax leftLift;
  SparkMax rightLift;
  SparkMax tilt;
  SparkMax vibrator;

  rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr safety_timer_;

  // ---------------- State ----------------
  bool enabled_         = false;  // true only while deadman held and not stopped
  bool stop_latched_    = false;  // set by B button or watchdog
  bool joy_lost_        = true;   // true until the first /joy message arrives
  bool slow_mode_       = START_IN_SLOW_MODE;
  bool vibrator_active_ = false;
  SteadyClock::time_point last_joy_time_{};

  // Previous button states, used for edge detection (act once per press)
  bool prev_deadman_         = false;
  bool prev_stop_button_     = false;
  bool prev_mode_button_     = false;
  bool prev_vibrator_button_ = false;

  // Last commanded values, used for the status log
  double left_speed_  = 0.0;
  double right_speed_ = 0.0;
  int lift_dir_ = 0;  // +1 up, -1 down, 0 stopped
  int tilt_dir_ = 0;

  // =================================================================
  //  SETUP
  // =================================================================

  void configure_drive_motor(SparkMax &motor, bool inverted)
  {
    motor.SetIdleMode(IdleMode::kBrake);
    motor.SetMotorType(MotorType::kBrushless);
    motor.SetSensorType(SensorType::kHallSensor);
    motor.SetInverted(inverted);

    motor.SetP(0, 0.0002f);
    motor.SetI(0, 0.0f);
    motor.SetD(0, 0.0f);
    motor.SetF(0, 0.00021f);

    // NEW: smooth acceleration and current protection
    motor.SetClosedLoopRampRate(DRIVE_RAMP_RATE);
    motor.SetSmartCurrentStallLimit(DRIVE_CURRENT_LIMIT);
    motor.SetSmartCurrentFreeLimit(DRIVE_CURRENT_LIMIT);
  }

  void configure_actuator(SparkMax &motor, bool inverted)
  {
    motor.SetIdleMode(IdleMode::kBrake);
    motor.SetMotorType(MotorType::kBrushed);
    motor.SetSensorType(SensorType::kEncoder);
    motor.SetInverted(inverted);

    motor.SetP(0, 1.51f);
    motor.SetI(0, 0.0f);
    motor.SetD(0, 0.0f);
    motor.SetF(0, 0.00021f);

    // NEW: smooth start/stop and current protection
    motor.SetRampRate(ACTUATOR_RAMP_RATE);
    motor.SetSmartCurrentStallLimit(ACTUATOR_CURRENT_LIMIT);
    motor.SetSmartCurrentFreeLimit(ACTUATOR_CURRENT_LIMIT);
  }

  void configure_motors()
  {
    configure_drive_motor(leftMotor, false);
    configure_drive_motor(rightMotor, true);   // right side is mirrored

    configure_actuator(leftLift, true);
    configure_actuator(rightLift, true);
    configure_actuator(tilt, true);

    vibrator.SetIdleMode(IdleMode::kBrake);
    vibrator.SetMotorType(MotorType::kBrushed);
    vibrator.SetSensorType(SensorType::kEncoder);
    vibrator.SetInverted(true);

    if (BURN_FLASH_ON_STARTUP) {
      leftMotor.BurnFlash();
      rightMotor.BurnFlash();
      leftLift.BurnFlash();
      rightLift.BurnFlash();
      tilt.BurnFlash();
      vibrator.BurnFlash();
    }
  }

  // =================================================================
  //  INPUT
  // =================================================================

  void joy_callback(const JoyMsg::SharedPtr msg)
  {
    last_joy_time_ = SteadyClock::now();

    // Crash protection: any error stops the motors instead of killing the node
    try {
      handle_input(*msg);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Error handling controller input: %s. Stopping all motors.", e.what());
      stop_latched_ = true;
      enabled_ = false;
      stop_all();
    }
  }

  void handle_input(const JoyMsg &msg)
  {
    // ---- 1. Deadman trigger and STOP button ----
    const bool deadman      = deadman_held(msg);
    const bool deadman_edge = deadman && !prev_deadman_;  // just squeezed
    prev_deadman_ = deadman;

    const bool stop_button = button(msg, Gp::_B);
    if (stop_button && !prev_stop_button_) {
      if (!stop_latched_) {
        RCLCPP_WARN(get_logger(),
          "STOP pressed: all motors stopped. Release and re-squeeze the deadman to resume.");
      }
      stop_latched_ = true;
    }
    prev_stop_button_ = stop_button;

    // A fresh squeeze of the deadman clears a STOP (but not while B is held)
    if (stop_latched_ && deadman_edge && !stop_button) {
      stop_latched_ = false;
      RCLCPP_INFO(get_logger(), "Stop cleared.");
    }

    // ---- 2. SLOW / FAST toggle (works while disabled so you can pick before driving) ----
    const bool mode_button = button(msg, Gp::_LEFT_BUMPER);
    if (mode_button && !prev_mode_button_) {
      slow_mode_ = !slow_mode_;
      RCLCPP_INFO(get_logger(), "Drive mode: %s", slow_mode_ ? "SLOW" : "FAST");
    }
    prev_mode_button_ = mode_button;

    // ---- 3. Enable / disable ----
    const bool should_enable = deadman && !stop_latched_;
    if (!should_enable) {
      if (enabled_ && !stop_latched_) {
        RCLCPP_INFO(get_logger(), "Deadman released: robot DISABLED.");
      }
      enabled_ = false;
      stop_all();  // re-sent every message so a lost CAN frame can't leave a motor running
      prev_vibrator_button_ = button(msg, Gp::_RIGHT_BUMPER);
      log_status();
      return;
    }

    if (!enabled_) {
      RCLCPP_INFO(get_logger(), "Deadman held: robot ENABLED (%s mode).", slow_mode_ ? "SLOW" : "FAST");
      enabled_ = true;
      send_heartbeat();  // wake the controllers right away
    }

    // ---- 4. Tank drive ----
    const double scale = slow_mode_ ? SLOW_MODE_SCALE : 1.0;
    const float left_stick  = std::clamp(axis(msg, Gp::_LEFT_VERTICAL_STICK),  -1.0f, 1.0f);
    const float right_stick = std::clamp(axis(msg, Gp::_RIGHT_VERTICAL_STICK), -1.0f, 1.0f);

    left_speed_  = left_stick  * VELOCITY_MAX * scale;
    right_speed_ = right_stick * VELOCITY_MAX * scale;

    leftMotor.SetVelocity(static_cast<float>(left_speed_));
    rightMotor.SetVelocity(static_cast<float>(right_speed_));

    // ---- 5. Lift (D-pad up/down) and tilt (D-pad left/right), move while held ----
    lift_dir_ = dpad_vertical(msg);
    tilt_dir_ = dpad_horizontal(msg);

    const float lift_output = lift_dir_ * LIFT_SPEED;
    leftLift.SetDutyCycle(lift_output);
    rightLift.SetDutyCycle(lift_output);   // both lifts move together
    tilt.SetDutyCycle(tilt_dir_ * TILT_SPEED);

    // ---- 6. Vibrator toggle (right bumper) ----
    const bool vib_button = button(msg, Gp::_RIGHT_BUMPER);
    if (vib_button && !prev_vibrator_button_) {
      vibrator_active_ = !vibrator_active_;
      RCLCPP_INFO(get_logger(), "Vibrator %s", vibrator_active_ ? "ON" : "OFF");
    }
    prev_vibrator_button_ = vib_button;
    vibrator.SetDutyCycle(vibrator_active_ ? VIBRATOR_OUTPUT : 0.0f);

    log_status();
  }

  // =================================================================
  //  SAFETY
  // =================================================================

  // Runs every HEARTBEAT_PERIOD: watchdog + heartbeat
  void safety_timer_callback()
  {
    const bool joy_fresh = (SteadyClock::now() - last_joy_time_) < JOY_TIMEOUT;

    if (!joy_fresh) {
      if (!joy_lost_) {
        joy_lost_ = true;
        RCLCPP_WARN(get_logger(),
          "No controller input for %lld ms: stopping all motors. "
          "Re-squeeze the deadman after it reconnects.",
          static_cast<long long>(std::chrono::milliseconds(JOY_TIMEOUT).count()));
        stop_latched_ = true;
        enabled_ = false;
        stop_all();
      }
      return;  // no heartbeat, so the controllers also disable themselves
    }

    if (joy_lost_) {
      joy_lost_ = false;
      RCLCPP_INFO(get_logger(), "Controller input received.");
    }

    // Heartbeats only while enabled: releasing the deadman lets the
    // SPARK MAX controllers time out and disable their outputs too.
    if (enabled_) {
      send_heartbeat();
    }
  }

  void send_heartbeat()
  {
    try {
     
      SparkBase::Heartbeat();
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Heartbeat failed: %s", e.what());
    }
  }

  // Commands every motor to zero. Each command is tried separately so one
  // failure can't skip the others. Never throws.
  void stop_all()
  {
    auto attempt = [](auto &&command) {
      try { command(); } catch (...) {}
    };
    // Duty cycle 0 (not velocity 0) so the drive stops immediately
    // instead of following the drive ramp rate.
    attempt([this] { leftMotor.SetDutyCycle(0.0f); });
    attempt([this] { rightMotor.SetDutyCycle(0.0f); });
    attempt([this] { leftLift.SetDutyCycle(0.0f); });
    attempt([this] { rightLift.SetDutyCycle(0.0f); });
    attempt([this] { tilt.SetDutyCycle(0.0f); });
    attempt([this] { vibrator.SetDutyCycle(0.0f); });

    vibrator_active_ = false;  // vibrator will not restart by itself
    left_speed_ = right_speed_ = 0.0;
    lift_dir_ = tilt_dir_ = 0;
  }

  // =================================================================
  //  HELPERS
  // =================================================================

  // Safe reads: a missing button/axis reads as "not pressed" instead of crashing
  static bool button(const JoyMsg &msg, int index)
  {
    return index >= 0 && static_cast<size_t>(index) < msg.buttons.size() && msg.buttons[index] != 0;
  }

  static float axis(const JoyMsg &msg, int index)
  {
    return (index >= 0 && static_cast<size_t>(index) < msg.axes.size()) ? msg.axes[index] : 0.0f;
  }

  static int direction_from_axis(float value)
  {
    if (value > 0.5f)  return 1;
    if (value < -0.5f) return -1;
    return 0;
  }

  static bool deadman_held(const JoyMsg &msg)
  {
    if (DEADMAN_IS_AXIS) {
      return axis(msg, DEADMAN_AXIS) < TRIGGER_PRESSED_BELOW;
    }
    return button(msg, DEADMAN_BUTTON);
  }

  // +1 = up, -1 = down, 0 = neither (or both)
  static int dpad_vertical(const JoyMsg &msg)
  {
    if (DPAD_IS_AXES) {
      return direction_from_axis(axis(msg, DPAD_VERTICAL_AXIS));
    }
    const bool up = button(msg, Gp::_D_PAD_UP);
    const bool down = button(msg, Gp::_D_PAD_DOWN);
    return (up == down) ? 0 : (up ? 1 : -1);
  }

  // +1 = left (tilt up), -1 = right (tilt down), 0 = neither
  static int dpad_horizontal(const JoyMsg &msg)
  {
    if (DPAD_IS_AXES) {
      return direction_from_axis(axis(msg, DPAD_HORIZONTAL_AXIS));
    }
    const bool left = button(msg, Gp::_D_PAD_LEFT);
    const bool right = button(msg, Gp::_D_PAD_RIGHT);
    return (left == right) ? 0 : (left ? 1 : -1);
  }

  static const char *direction_name(int dir)
  {
    return dir > 0 ? "UP" : (dir < 0 ? "DOWN" : "-");
  }

  void log_status()
  {
    const char *state = stop_latched_ ? "STOPPED" : (enabled_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "%s | %s | L: %.0f RPM | R: %.0f RPM | Lift: %s | Tilt: %s | Vibrator: %s",
      state, slow_mode_ ? "SLOW" : "FAST", left_speed_, right_speed_,
      direction_name(lift_dir_), direction_name(tilt_dir_), vibrator_active_ ? "ON" : "OFF");
  }
};

// =====================================================================
//  MAIN
// =====================================================================

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  const std::string can_interface = "can0";  // use "vcan0" for practice without the robot

  std::shared_ptr<ControllerNode> node;
  try {
    node = std::make_shared<ControllerNode>(can_interface);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("controller_node"),
      "Could not start (is %s up and wired?): %s", can_interface.c_str(), e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  node.reset();  // runs the destructor, which stops every motor
  rclcpp::shutdown();
  return 0;
}
