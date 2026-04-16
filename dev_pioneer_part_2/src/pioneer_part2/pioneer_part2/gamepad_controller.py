"""
Gamepad controller node for Pioneer P3-AT Part 2.

Implements the Bluetooth gamepad interface specified in the task:
  - X button  → enable automated mode
  - O button  → enable manual mode
  - Back pedals (L2+R2 triggers) → dead-man switch in automated mode
  - Left stick → manual drive (forward/back + turn)

Publishes
---------
  /gamepad_mode  std_msgs/String   "auto" | "manual" | "stopped"
  /cmd_vel       geometry_msgs/Twist   (manual mode only, or stop command)

Subscribes
----------
  /joy   sensor_msgs/Joy

When in automated mode and the dead-man triggers are released the node
immediately publishes a zero Twist to /cmd_vel to stop the robot.

Button / Axis mapping (PlayStation DS4 via python-joy / joystick driver):
  buttons[0]  → Cross (X)
  buttons[1]  → Circle (O)
  axes[2]     → L2 trigger  (-1=released, +1=fully pressed)
  axes[5]     → R2 trigger  (-1=released, +1=fully pressed)
  axes[1]     → left stick Y (forward = +1)
  axes[0]     → left stick X (left = +1)

Note: button/axis indices can be overridden via ROS parameters.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    from sensor_msgs.msg import Joy
    _JOY_AVAILABLE = True
except ImportError:
    _JOY_AVAILABLE = False


class GamepadController(Node):

    MODE_MANUAL = 'manual'
    MODE_AUTO = 'auto'
    MODE_STOPPED = 'stopped'

    def __init__(self):
        super().__init__('gamepad_controller')

        self.declare_parameter('auto_button', 0)
        self.declare_parameter('manual_button', 1)
        self.declare_parameter('deadman_axis_l', 2)
        self.declare_parameter('deadman_axis_r', 5)
        self.declare_parameter('deadman_threshold', 0.5)
        self.declare_parameter('drive_linear_axis', 1)
        self.declare_parameter('drive_angular_axis', 0)
        self.declare_parameter('max_manual_linear', 0.35)
        self.declare_parameter('max_manual_angular', 1.2)

        self._auto_btn = self.get_parameter('auto_button').value
        self._manual_btn = self.get_parameter('manual_button').value
        self._dm_axis_l = self.get_parameter('deadman_axis_l').value
        self._dm_axis_r = self.get_parameter('deadman_axis_r').value
        self._dm_thresh = self.get_parameter('deadman_threshold').value
        self._lin_axis = self.get_parameter('drive_linear_axis').value
        self._ang_axis = self.get_parameter('drive_angular_axis').value
        self._max_lin = self.get_parameter('max_manual_linear').value
        self._max_ang = self.get_parameter('max_manual_angular').value

        self._mode = self.MODE_STOPPED
        self._deadman_held = False

        self._mode_pub = self.create_publisher(String, 'gamepad_mode', 10)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        if _JOY_AVAILABLE:
            self.create_subscription(Joy, 'joy', self._joy_cb, 10)
            self.get_logger().info(
                'Gamepad controller ready — waiting for joystick input on /joy\n'
                f'  X (btn {self._auto_btn}) = auto mode\n'
                f'  O (btn {self._manual_btn}) = manual mode\n'
                f'  L2/R2 (axes {self._dm_axis_l}/{self._dm_axis_r}) = dead-man switch')
        else:
            self.get_logger().warn(
                'sensor_msgs.Joy not available — gamepad disabled')

        # Publish initial mode
        self._publish_mode()

        # Periodic heartbeat so mission controller always knows the current mode
        self.create_timer(0.5, self._heartbeat)

    # ---- joy callback --------------------------------------------------------

    def _joy_cb(self, msg: 'Joy'):
        buttons = msg.buttons
        axes = msg.axes

        def _btn(idx):
            return bool(buttons[idx]) if idx < len(buttons) else False

        def _axis(idx):
            return axes[idx] if idx < len(axes) else 0.0

        # --- Mode switching ---
        if _btn(self._auto_btn):
            if self._mode != self.MODE_AUTO:
                self._mode = self.MODE_AUTO
                self.get_logger().info('Mode: AUTOMATED')
                self._publish_mode()

        if _btn(self._manual_btn):
            if self._mode != self.MODE_MANUAL:
                self._mode = self.MODE_MANUAL
                self.get_logger().info('Mode: MANUAL')
                self._publish_mode()

        # --- Dead-man switch (auto mode) ---
        if self._mode == self.MODE_AUTO:
            l2 = _axis(self._dm_axis_l)
            r2 = _axis(self._dm_axis_r)
            # Triggers: -1 = released, +1 = fully pressed
            # Treat as held if either trigger exceeds threshold
            held = (l2 > self._dm_thresh) or (r2 > self._dm_thresh)
            if held != self._deadman_held:
                self._deadman_held = held
                if not held:
                    self.get_logger().info(
                        'Dead-man switch released — stopping robot')
                    self._cmd_pub.publish(Twist())
                    self._publish_mode()  # re-announce mode
                else:
                    self.get_logger().info('Dead-man switch engaged')

        # --- Manual driving ---
        if self._mode == self.MODE_MANUAL:
            lin = _axis(self._lin_axis) * self._max_lin
            ang = _axis(self._ang_axis) * self._max_ang
            cmd = Twist()
            cmd.linear.x = lin
            cmd.angular.z = ang
            self._cmd_pub.publish(cmd)

    # ---- helpers -------------------------------------------------------------

    def _publish_mode(self):
        msg = String()
        if self._mode == self.MODE_AUTO and not self._deadman_held:
            msg.data = self.MODE_STOPPED
        else:
            msg.data = self._mode
        self._mode_pub.publish(msg)

    def _heartbeat(self):
        self._publish_mode()

    @property
    def is_auto_active(self) -> bool:
        """True when in auto mode AND dead-man switch is held."""
        return self._mode == self.MODE_AUTO and self._deadman_held

    @property
    def current_mode(self) -> str:
        return self._mode


def main(args=None):
    rclpy.init(args=args)
    node = GamepadController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
