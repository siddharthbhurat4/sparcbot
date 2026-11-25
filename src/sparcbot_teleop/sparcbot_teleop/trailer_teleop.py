"""Keyboard teleop for SPARCBot with a trailer: only commands the trailer can follow.

    ros2 launch sparcbot_bringup robot.launch.py trailer_sensor:=true   # terminal 1
    ros2 run sparcbot_teleop trailer_teleop                             # terminal 2

Run it with `ros2 run` (not from a launch file): it needs the terminal's keyboard.

Keys. A terminal cannot see two keys held at once, so the keys LATCH: tap one to set the
direction, tap another to set the steering, and they combine.

  w : forward              a : steer left (centre of the circle on the left)
  s : reverse              d : steer right
  (tap a then d, or d then a, to go back to straight)

  + / = : speed up one level     - : slow down one level     (3 speed levels)
  > / . : turn rate up one level  < / , : turn rate down one level  (3 turn levels)
  space : stop/coast     b : brake     r : reset encoders/controllers     q : quit
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger

from sparcbot_teleop.keyboard_teleop import RawTerminal


# ======================================================================
# SETTINGS: everything you may want to change is here
# ======================================================================

# The trailer: MEASURE THESE (same numbers as in trailer_lqr.py).
TRAILER_LENGTH_M = 0.081         # L: hitch pivot to the centre of the trailer's axle [m]
HITCH_OFFSET_M = 0.14           # a: how far BEHIND the robot's axle centre the hitch pivot is [m]

# The three speeds [m/s] and three turn rates [rad/s] (magnitudes; the keys set the signs).
SPEED_LEVELS_M_S = (0.10, 0.20, 0.30)
TURN_LEVELS_RAD_S = (0.10, 0.20, 0.30)
START_SPEED_LEVEL = 0           # index into SPEED_LEVELS_M_S
START_TURN_LEVEL = 0            # index into TURN_LEVELS_RAD_S

# A combination is refused if its steady trailer angle is bigger than this. Keep it a little
# under trailer_lqr's MAX_TRAILER_ANGLE_DEG (40) so there is room left to correct errors.
MAX_STEADY_TRAILER_ANGLE_DEG = 60.0

SPEED_NAMES = ('slow', 'medium', 'fast')
TURN_NAMES = ('gentle', 'medium', 'sharp')

HELP = """
Trailer Teleop (only commands the trailer can follow)
=====================================================
  w : forward     a : steer left      + / = : speed up        > / . : turn rate up
  s : reverse     d : steer right     -     : slow down       < / , : turn rate down
  (a then d, or d then a, = straight)
  space : stop/coast     b : brake     r : reset encoders/controllers     q : quit
"""


# ----------------------------------------------------------------------
# Feasibility
# ----------------------------------------------------------------------

def steady_trailer_angle(v, omega, length_m, hitch_offset_m):
    """Steady trailer angle phi* [rad] on the circle (v, omega), or None if there is none.

    Solves  v*sin(phi) + a*omega*cos(phi) + L*omega = 0  for the solution nearest 0, using
    sin(phi) + c*cos(phi) = sqrt(1 + c^2) * sin(phi + atan(c)),  c = a*omega/v.
    """
    if v == 0.0:
        return None                         # not a circle
    k = omega / v
    c = hitch_offset_m * k
    s = -length_m * k / math.sqrt(1.0 + c * c)
    if abs(s) > 1.0:
        return None                         # circle too tight: the trailer folds
    return math.asin(s) - math.atan(c)


def is_feasible(v, omega, length_m=TRAILER_LENGTH_M, hitch_offset_m=HITCH_OFFSET_M,
                max_angle_deg=MAX_STEADY_TRAILER_ANGLE_DEG):
    phi = steady_trailer_angle(v, omega, length_m, hitch_offset_m)
    return phi is not None and abs(math.degrees(phi)) <= max_angle_deg


# ======================================================================
# ROS node
# ======================================================================

class TrailerTeleop(Node):

    def __init__(self):
        super().__init__('trailer_teleop')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)

        self.coast_client = self.create_client(Trigger, 'wheel_controller/coast')
        self.brake_client = self.create_client(Trigger, 'wheel_controller/brake')
        self.reset_controller_client = self.create_client(Trigger, 'wheel_controller/reset')
        self.reset_encoders_client = self.create_client(Trigger, 'sparcbot_driver/reset_encoders')

        self.speed_level = START_SPEED_LEVEL
        self.turn_level = START_TURN_LEVEL
        self.direction = 0                  # +1 forward, -1 reverse, 0 stopped
        self.steer = 0                      # +1 centre on the left, -1 on the right, 0 straight

        self.print_feasibility_table()

    # ------------------------------------------------------------------

    def print_feasibility_table(self):
        print(f'Trailer: L = {TRAILER_LENGTH_M:.3f} m, hitch offset a = {HITCH_OFFSET_M:.3f} m, '
              f'steady trailer angle limit {MAX_STEADY_TRAILER_ANGLE_DEG:.0f} deg.')
        print('Steady trailer angle on each arc (-- = refused). Rows: speed, columns: turn rate.')
        header = '          ' + ''.join(f'{t:>7.2f}' for t in TURN_LEVELS_RAD_S) + '  rad/s'
        print(header)
        for i, v in enumerate(SPEED_LEVELS_M_S):
            cells = []
            for w in TURN_LEVELS_RAD_S:
                phi = steady_trailer_angle(v, w, TRAILER_LENGTH_M, HITCH_OFFSET_M)
                ok = is_feasible(v, w)
                cells.append(f'{abs(math.degrees(phi)):6.1f}°' if ok else '     --')
            print(f'  {v:5.2f} m/s' + ''.join(cells) + f'   ({SPEED_NAMES[i]})')
        if not any(is_feasible(v, w) for v in SPEED_LEVELS_M_S for w in TURN_LEVELS_RAD_S):
            print('WARNING: no arc is allowed with these settings. Only straight driving works.')
        print()

    # ------------------------------------------------------------------

    @staticmethod
    def command(direction, steer, speed_level, turn_level):
        """(v, w) for a direction, steering and levels. Both are 0 if stopped."""
        v = direction * SPEED_LEVELS_M_S[speed_level]
        # The centre of the circle stays on the same side in both directions, so in reverse
        # (direction = -1) the sign of w flips.
        w = steer * direction * TURN_LEVELS_RAD_S[turn_level]
        return v, w

    def describe(self, direction=None, steer=None, speed_level=None, turn_level=None):
        direction = self.direction if direction is None else direction
        steer = self.steer if steer is None else steer
        speed_level = self.speed_level if speed_level is None else speed_level
        turn_level = self.turn_level if turn_level is None else turn_level
        motion = {1: 'forward', -1: 'reverse', 0: 'stopped'}[direction]
        side = {1: ' + steer left', -1: ' + steer right', 0: ''}[steer]
        return (f'{motion}{side} | speed {SPEED_NAMES[speed_level]} '
                f'({SPEED_LEVELS_M_S[speed_level]:g} m/s), turn {TURN_NAMES[turn_level]} '
                f'({TURN_LEVELS_RAD_S[turn_level]:g} rad/s)')

    # ------------------------------------------------------------------

    def publish(self, linear_m_s, angular_rad_s):
        msg = Twist()
        msg.linear.x = float(linear_m_s)
        msg.angular.z = float(angular_rad_s)
        self.cmd_vel_pub.publish(msg)

    def call(self, client, name):
        if not client.service_is_ready():
            self.get_logger().warn(f'{name} service not available (is the robot launch running?)')
            return None
        return client.call_async(Trigger.Request())

    def change(self, direction=None, steer=None, speed_level=None, turn_level=None):
        """Apply a change if the trailer can follow the result; otherwise refuse it."""
        new = (self.direction if direction is None else direction,
               self.steer if steer is None else steer,
               self.speed_level if speed_level is None else speed_level,
               self.turn_level if turn_level is None else turn_level)

        v, w = self.command(*new)
        if new[0] != 0 and not is_feasible(v, w):
            print(f'\nRefused: {self.describe(*new)} is too tight for the trailer. '
                  f'Still: {self.describe()}.')
            return False

        self.direction, self.steer, self.speed_level, self.turn_level = new
        if self.direction != 0:
            self.publish(v, w)
            print(f'\n{self.describe()}: v={v:+.2f} m/s  w={w:+.2f} rad/s')
        else:
            print(f'\n{self.describe()}')
        return True

    def stop_commands(self):
        """Zero /cmd_vel first (clears any latched command), then the service."""
        self.direction = 0
        self.steer = 0
        self.publish(0.0, 0.0)

    def coast(self):
        self.stop_commands()
        return self.call(self.coast_client, 'wheel_controller/coast')

    # ------------------------------------------------------------------

    def handle_key(self, key):
        """Return False when the user asked to quit."""
        key = key.lower() if key.isalpha() else key
        top = len(SPEED_LEVELS_M_S) - 1
        if key == 'w':
            self.change(direction=+1)
        elif key == 's':
            self.change(direction=-1)
        elif key == 'a':
            self.change(steer=min(+1, self.steer + 1))
        elif key == 'd':
            self.change(steer=max(-1, self.steer - 1))
        elif key in ('+', '='):
            self.change(speed_level=min(top, self.speed_level + 1))
        elif key in ('-', '_'):
            self.change(speed_level=max(0, self.speed_level - 1))
        elif key in ('>', '.'):
            self.change(turn_level=min(len(TURN_LEVELS_RAD_S) - 1, self.turn_level + 1))
        elif key in ('<', ','):
            self.change(turn_level=max(0, self.turn_level - 1))
        elif key == ' ':
            self.coast()
            print('\nStopped (coasting).')
        elif key == 'b':
            self.stop_commands()
            self.call(self.brake_client, 'wheel_controller/brake')
            print('\nStopped (braking).')
        elif key == 'r':
            self.stop_commands()
            self.call(self.reset_encoders_client, 'sparcbot_driver/reset_encoders')
            self.call(self.reset_controller_client, 'wheel_controller/reset')
            print('\nEncoders and controllers reset.')
        elif key == 'q':
            return False
        return True


def main(args=None):
    # Handle Ctrl-C ourselves so ROS is still up to send the final coast.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TrailerTeleop()
    print(HELP)
    print(node.describe())

    try:
        with RawTerminal() as terminal:
            running = True
            while running and rclpy.ok():
                key = terminal.read_key()
                if key is not None:
                    running = node.handle_key(key)
                rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            future = node.coast()
            if future is not None:
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            print('\nStopped. Both motors are coasting.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
