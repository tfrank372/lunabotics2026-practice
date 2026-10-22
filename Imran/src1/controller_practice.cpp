#include "SparkMax.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

// ---------------- Tunable constants ----------------
const float VELOCITY_MAX = 2500.0f;     // drive motor RPM at full stick (same as team code)
const float SLOW_FACTOR = 0.5f;         // arcade mode runs at half speed for fine moves
const float VIBRATOR_OUTPUT = 1.0f;
const float ACTUATOR_OUTPUT = 1.0f;
const float JOYSTICK_DEADBAND = 0.05f;  // ignore stick drift near the center

// Browser gamepads report stick UP as -1, so the team code flips the sign.
// CHECK ON THE BENCH (robot on blocks): if it drives backwards, set this to +1.
const float DRIVE_SIGN = -1.0f;

// If no /joy message arrives for this long, everything stops and heartbeats stop.
const double JOY_TIMEOUT_S = 0.2;

// Control loop period: commands and heartbeats are sent from here, not from the callback.
const int CONTROL_PERIOD_MS = 20;  // 50 Hz

// BurnFlash writes settings to the SPARK flash memory. Do it once after changing
// the configuration, not on every start.
const bool BURN_FLASH = false;

// Team code has 7 controllers (two tilt actuators). The practice README lists 6.
enum CAN_IDs
{
    LEFT_MOTOR = 1,
    RIGHT_MOTOR = 2,
    LEFT_LIFT = 3,
    RIGHT_LIFT = 4,
    LEFT_TILT = 5,
    VIBRATOR = 6,
    RIGHT_TILT = 7,
};

namespace Gp
{
    enum Buttons
    {
        _A = 0,              // Excavation autonomy (placeholder)
        _B = 1,              // Stop automation (placeholder)
        _X = 2,              // Excavation reset (placeholder)
        _Y = 3,              // Deposit autonomy (placeholder)
        _LEFT_BUMPER = 4,    // Switch drive mode
        _RIGHT_BUMPER = 5,   // Vibrator toggle
        _LEFT_TRIGGER = 6,   // Safety trigger
        _RIGHT_TRIGGER = 7,  // Safety trigger
        _D_PAD_UP = 12,      // Lift up
        _D_PAD_DOWN = 13,    // Lift down
        _D_PAD_LEFT = 14,    // Tilt up   (CHECK direction)
        _D_PAD_RIGHT = 15,   // Tilt down (CHECK direction)
        _BUTTON_COUNT = 16,  // need at least buttons 0..15
    };

    enum Axes
    {
        _LEFT_HORIZONTAL_STICK = 0,
        _LEFT_VERTICAL_STICK = 1,
        _RIGHT_HORIZONTAL_STICK = 2,
        _RIGHT_VERTICAL_STICK = 3,
        _AXIS_COUNT = 4,
    };
}

// The latest operator request. The joy callback only fills this in;
// the control loop decides what actually goes out on the CAN bus.
struct Command
{
    bool armed = false;  // a safety trigger is held
    float left_drive = 0.0f;
    float right_drive = 0.0f;
    float lift = 0.0f;
    float tilt = 0.0f;
};

class ControllerNode : public rclcpp::Node
{
public:
    explicit ControllerNode(const std::string &can_interface)
        : Node("imran_controller_node"),
          leftMotor(can_interface, LEFT_MOTOR),
          rightMotor(can_interface, RIGHT_MOTOR),
          leftLift(can_interface, LEFT_LIFT),
          rightLift(can_interface, RIGHT_LIFT),
          leftTilt(can_interface, LEFT_TILT),
          rightTilt(can_interface, RIGHT_TILT),
          vibrator(can_interface, VIBRATOR),
          last_joy_time_(this->now())
    {
        RCLCPP_INFO(this->get_logger(), "Initializing motor controllers");
        configure_motors();
        RCLCPP_INFO(this->get_logger(), "Motor controllers initialized");

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ControllerNode::joy_callback, this, std::placeholders::_1));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(CONTROL_PERIOD_MS),
            std::bind(&ControllerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Controller node ready. Hold LT or RT to drive.");
    }

private:
    SparkMax leftMotor;
    SparkMax rightMotor;
    SparkMax leftLift;
    SparkMax rightLift;
    SparkMax leftTilt;
    SparkMax rightTilt;
    SparkMax vibrator;

    Command cmd_;
    rclcpp::Time last_joy_time_;
    bool have_joy_ = false;

    bool arcade_mode_ = false;
    bool vibrator_active_ = false;
    bool prev_mode_button_ = false;
    bool prev_vibrator_button_ = false;
    bool prev_a_ = false, prev_b_ = false, prev_x_ = false, prev_y_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ---------------- Setup ----------------
    void configure_motors()
    {
        for (SparkMax *m : {&leftMotor, &rightMotor})
        {
            m->SetIdleMode(IdleMode::kBrake);
            m->SetMotorType(MotorType::kBrushless);
            m->SetSensorType(SensorType::kHallSensor);
            m->SetP(0, 0.0002f);
            m->SetI(0, 0.0f);
            m->SetD(0, 0.0f);
            m->SetF(0, 0.00021f);
        }
        leftMotor.SetInverted(false);
        rightMotor.SetInverted(true);

        for (SparkMax *m : {&leftLift, &rightLift, &leftTilt, &rightTilt, &vibrator})
        {
            m->SetIdleMode(IdleMode::kBrake);
            m->SetMotorType(MotorType::kBrushed);
            m->SetSensorType(SensorType::kEncoder);
            m->SetInverted(true);
        }

        // TODO: current limits. The API exists (SetSmartCurrentStallLimit /
        // SetSmartCurrentFreeLimit) but the team has not chosen the values yet,
        // so none are set here. Do not guess them.

        if (BURN_FLASH)
        {
            for (SparkMax *m : all_motors())
                m->BurnFlash();
        }
    }

    std::array<SparkMax *, 7> all_motors()
    {
        return {&leftMotor, &rightMotor, &leftLift, &rightLift, &leftTilt, &rightTilt, &vibrator};
    }

    // ---------------- Helpers ----------------
    static float apply_deadband(float value)
    {
        float clamped = std::clamp(value, -1.0f, 1.0f);
        return (std::fabs(clamped) < JOYSTICK_DEADBAND) ? 0.0f : clamped;
    }

    static float button_pair_output(bool positive, bool negative)
    {
        return (static_cast<float>(positive) - static_cast<float>(negative)) * ACTUATOR_OUTPUT;
    }

    // True only at the moment the button goes from released to pressed.
    static bool rising_edge(bool now, bool &prev)
    {
        bool edge = now && !prev;
        prev = now;
        return edge;
    }

    // Every CAN write goes through here, so a full bus (e.g. E-stop pressed)
    // logs an error instead of killing the node.
    template <typename F>
    bool can_safe(const char *what, F &&fn)
    {
        try
        {
            fn();
            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "CAN error (%s): %s", what, e.what());
            return false;
        }
    }

    // ---------------- Joy callback: read buttons, store the request ----------------
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->axes.size() < Gp::_AXIS_COUNT || msg->buttons.size() < Gp::_BUTTON_COUNT)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Joy message has too few axes/buttons (%zu axes, %zu buttons)",
                                 msg->axes.size(), msg->buttons.size());
            return;
        }

        last_joy_time_ = this->now();
        have_joy_ = true;

        auto pressed = [&msg](int b) { return msg->buttons[b] != 0; };

        cmd_.armed = pressed(Gp::_LEFT_TRIGGER) || pressed(Gp::_RIGHT_TRIGGER);

        // Toggles work only while armed, so a bumped controller does nothing.
        bool mode_edge = rising_edge(pressed(Gp::_LEFT_BUMPER), prev_mode_button_);
        bool vib_edge = rising_edge(pressed(Gp::_RIGHT_BUMPER), prev_vibrator_button_);
        if (cmd_.armed && mode_edge)
        {
            arcade_mode_ = !arcade_mode_;
            RCLCPP_INFO(this->get_logger(), "Drive mode: %s", arcade_mode_ ? "ARCADE (slow)" : "TANK");
        }
        if (cmd_.armed && vib_edge)
        {
            vibrator_active_ = !vibrator_active_;
            RCLCPP_INFO(this->get_logger(), "Vibrator %s", vibrator_active_ ? "ON" : "OFF");
        }

        // Drive
        if (arcade_mode_)
        {
            float forward = DRIVE_SIGN * apply_deadband(msg->axes[Gp::_LEFT_VERTICAL_STICK]);
            float turn = apply_deadband(msg->axes[Gp::_LEFT_HORIZONTAL_STICK]);
            cmd_.left_drive = std::clamp(forward + turn, -1.0f, 1.0f) * SLOW_FACTOR;
            cmd_.right_drive = std::clamp(forward - turn, -1.0f, 1.0f) * SLOW_FACTOR;
        }
        else
        {
            cmd_.left_drive = DRIVE_SIGN * apply_deadband(msg->axes[Gp::_LEFT_VERTICAL_STICK]);
            cmd_.right_drive = DRIVE_SIGN * apply_deadband(msg->axes[Gp::_RIGHT_VERTICAL_STICK]);
        }

        // Actuators
        cmd_.lift = button_pair_output(pressed(Gp::_D_PAD_UP), pressed(Gp::_D_PAD_DOWN));
        cmd_.tilt = button_pair_output(pressed(Gp::_D_PAD_LEFT), pressed(Gp::_D_PAD_RIGHT));

        // Autonomy buttons: placeholders until the action servers are available
        // (they need msg_pkg from the team workspace).
        if (rising_edge(pressed(Gp::_A), prev_a_))
            RCLCPP_INFO(this->get_logger(), "A: would start excavation autonomy");
        if (rising_edge(pressed(Gp::_Y), prev_y_))
            RCLCPP_INFO(this->get_logger(), "Y: would start deposit autonomy");
        if (rising_edge(pressed(Gp::_X), prev_x_))
            RCLCPP_INFO(this->get_logger(), "X: would home lift and tilt");
        if (rising_edge(pressed(Gp::_B), prev_b_))
            RCLCPP_INFO(this->get_logger(), "B: would cancel all autonomy goals");
    }

    // ---------------- Control loop: the only place that writes to CAN ----------------
    void control_loop()
    {
        bool fresh = have_joy_ && (this->now() - last_joy_time_).seconds() < JOY_TIMEOUT_S;

        if (!fresh)
        {
            // Lost the controller: no heartbeat, so the SPARKs disable themselves.
            // Also clear toggles so nothing restarts by surprise when the link returns.
            vibrator_active_ = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No fresh /joy: outputs stopped, heartbeat paused");
            return;
        }

        bool armed = cmd_.armed;
        float left = armed ? cmd_.left_drive * VELOCITY_MAX : 0.0f;
        float right = armed ? cmd_.right_drive * VELOCITY_MAX : 0.0f;
        float lift = armed ? cmd_.lift : 0.0f;
        float tilt = armed ? cmd_.tilt : 0.0f;
        float vib = (armed && vibrator_active_) ? VIBRATOR_OUTPUT : 0.0f;

        // Heartbeat is a broadcast to all SPARKs, so one call is enough.
        can_safe("heartbeat", [] { SparkBase::Heartbeat(); });

        if (armed)
        {
            can_safe("drive", [&] { leftMotor.SetVelocity(left); rightMotor.SetVelocity(right); });
        }
        else
        {
            // Duty cycle 0 stops at once; velocity 0 would let the PID fight to hold.
            can_safe("drive", [&] { leftMotor.SetDutyCycle(0.0f); rightMotor.SetDutyCycle(0.0f); });
        }
        can_safe("lift", [&] { leftLift.SetDutyCycle(lift); rightLift.SetDutyCycle(lift); });
        can_safe("tilt", [&] { leftTilt.SetDutyCycle(tilt); rightTilt.SetDutyCycle(tilt); });
        can_safe("vibrator", [&] { vibrator.SetDutyCycle(vib); });

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "%s | %s | L %.0f RPM | R %.0f RPM | Lift %.1f | Tilt %.1f | Vib %s",
            armed ? "ARMED" : "SAFE", arcade_mode_ ? "ARCADE" : "TANK",
            left, right, lift, tilt, vib > 0.0f ? "ON" : "OFF");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerNode>("can0");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
