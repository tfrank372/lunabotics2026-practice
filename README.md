# lunabotics2026-practice
Members practice writing their own files to have put on the robot.

helpful notes:
enum CAN_IDs { LEFT_MOTOR = 1, RIGHT_MOTOR = 2, LEFT_LIFT = 3, RIGHT_LIFT = 4, TILT = 5, VIBRATOR = 6 };

namespace Gp { enum Buttons { _A = 0, // Excavation Autonomy _B = 1, // Stop Automation _X = 2, // Excavation Reset _Y = 3, // Deposit Autonomy _LEFT_BUMPER = 4, // Alternate between control modes _RIGHT_BUMPER = 5, // Vibration Toggle _LEFT_TRIGGER = 6, // Safety Trigger _RIGHT_TRIGGER = 7, // Safety Trigger _WINDOW_KEY = 8, // Button 8 /** I do not know what else to call this key / _D_PAD_UP = 12, // Lift Actuator UP _D_PAD_DOWN = 13, // Lift Actuator DOWN _D_PAD_LEFT = 14, // Tilt Actuator Up /* CHECK THESE / _D_PAD_RIGHT = 15, // Tilt Actuator Down /* CHECK THESE */ _X_BOX_KEY = 16 };
