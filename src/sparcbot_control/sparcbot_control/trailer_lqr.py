import math
import sys
import time
from math import sqrt
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions

from sparcbot_interfaces.msg import TrailerAngle

# The trailer: measured on the robot.
L = 0.081                               # L: hitch pivot to the centre of the trailer's axle [m]
a = 0.14                               # a: how far BEHIND the robot's axle centre the pivot is [m]

Q = 1.0                                 # cost on the trailer-angle error
R = 1.0                                 # cost on the steering correction

SPEED_LEVELS = [i / 10 for i in range(-4, 5)]    # [m/s]
TURN_LEVELS = [j / 10 for j in range(-5, 6)]     # [rad/s]

MAX_STEADY_ANGLE_DEG = 90.0             # arcs with a bigger steady trailer angle are not in the table
RATE_HZ = 50.0                  # control rate
MAX_ANGULAR_RAD_S = 2.0         # limit on the angular.z we publish
MAX_TRAILER_ANGLE_DEG = 80.0    # stop if |phi| is bigger than this (jackknife is near)
TRAILER_TIMEOUT_S = 0.2         # stop if /trailer_angle is silent for this long


# TODO: replace hitch_equilibrium_angle(v_eq, omega_eq) with working code from simulation
def hitch_equilibrium_angle(v_eq, omega_eq):
    phi_eq = None 
    return phi_eq

# TODO: replace linearize(v_eq, omega_eq) with working code from simuation 
def linearize(v_eq, omega_eq):
    phi_eq = hitch_equilibrium_angle(v_eq, omega_eq)
    A = 0 
    Bphi = 0 
    return phi_eq, A, Bphi

# TODO: scalar_lqr(A, Bphi, Q, R) with working code from simuation 
def scalar_lqr(A, Bphi, Q, R):
    K = 0
    return K 

# TODO: build_table(Q, R) with working code from simuation 
def build_table(Q, R):
    gain_table = np.zeros((len(SPEED_LEVELS), len(TURN_LEVELS)))
    for i, v_eq in enumerate(SPEED_LEVELS):
        for j, omega_eq in enumerate(TURN_LEVELS):
            gain_table[i,j] = 0 
    return gain_table

# TODO: gain_scheduled_controller(gain_table, v_cmd, omega_cmd, phi_measured) with working code from simuation 
def gain_scheduled_controller(gain_table, v_cmd, omega_cmd, phi_measured):
    i = SPEED_LEVELS.index(v_cmd)
    j = TURN_LEVELS.index(omega_cmd)
    u = 0 
    return u


def snap_to_level(levels, value, tol=1e-3):
    """The entry of `levels` within tol of `value` (so 0.2000001 finds 0.2), or None."""
    for level in levels:
        if abs(level - value) < tol:
            return level
    return None


def command_to_send(gain_table, v_cmd, omega_cmd, phi_measured):
    """What the node publishes for a teleop command: (omega_out, phi_eq), or None to refuse.

    None means: reversing with a command we have no gain for, so the node must stop.
    phi_eq is None when the command is passed through unchanged (forward and not in the table).
    """
    v_eq = snap_to_level(SPEED_LEVELS, v_cmd)
    omega_eq = snap_to_level(TURN_LEVELS, omega_cmd)

    phi_eq = None
    if v_eq is not None and omega_eq is not None:
        phi_eq = hitch_equilibrium_angle(v_eq, omega_eq)

    if phi_eq is None or abs(phi_eq) > math.radians(MAX_STEADY_ANGLE_DEG):
        if v_cmd < 0.0:
            return None                 # reverse is unstable: no gain, no driving
        return omega_cmd, None          # forward: the trailer is stable on its own

    u = gain_scheduled_controller(gain_table, v_eq, omega_eq, phi_measured)
    return max(-MAX_ANGULAR_RAD_S, min(MAX_ANGULAR_RAD_S, u)), phi_eq


def run_tests():
    global L, a
    failures = []

    def check(name, got, expected, tol=5e-4):
        ok = got is not None and abs(got - expected) <= tol
        print("\t" f"{'PASS' if ok else 'FAIL'}  {name}: got {got if got is None else round(got, 4)}, "
            f"expected {expected}")
        if not ok:
            failures.append(name)

    def check_true(name, condition):
        print("\t" f"{'PASS' if condition else 'FAIL'}  {name}")
        if not condition:
            failures.append(name)

    print('TEST 1: hitch_equilibrium_angle()')
    for v, w, expected in [(0.2, 0.0, 0.0), (-0.2, 0.3, 18.69), (-0.1, -0.1, -12.57),
                        (0.2, -0.3, 18.69), (0.1, 0.3, -35.72)]:
        phi_eq = hitch_equilibrium_angle(v, w)
        check(f'v={v:+.2f} w={w:+.2f}', None if phi_eq is None else math.degrees(phi_eq), expected, 0.01)
    check_true('v = 0 has no circle', hitch_equilibrium_angle(0.0, 0.3) is None)
    L, a = 0.30, 0.10
    check_true('too tight of a circle has no angle (L=0.30, a=0.10, v=0.1, w=1.0)',
            hitch_equilibrium_angle(0.1, 1.0) is None)
    L, a = 0.081, 0.14
    print("\t---------------------------")

    print('\n\n\nTEST 2: linearize()')
    for v, w, phi_deg, A, Bphi in [
            (-0.20, 0.0, 0.0, 2.4691, -2.7284),
            (-0.20, 0.3, 18.69, 2.5051, -2.6373),
            (-0.15, 0.2, 16.67, 1.8732, -2.6558),
            (-0.10, -0.1, -12.57, 1.2426, -2.6870),
            (0.10, 0.0, 0.0, -1.2346, -2.7284),
            (0.20, -0.3, 18.69, -2.5051, -2.6373)]:
        out = linearize(v, w)
        tag = f'v={v:+.2f} w={w:+.2f}'
        check(tag + ' phi*', None if out[0] is None else math.degrees(out[0]), phi_deg, 0.01)
        check(tag + ' A', None if out[0] is None else out[1], A)
        check(tag + ' B', None if out[0] is None else out[2], Bphi)
    print("\t---------------------------")

    print('\n\n\nTEST 3: scalar_lqr()')
    Q = 1
    R = 1
    print("\n\tFor Q = 1, R = 1.")
    for v, w, phi_deg, A, Bphi, K in [
            (-0.20, 0.0, 0.0, 2.4691, -2.7284, -2.2537),
            (-0.20, 0.3, 18.69, 2.5051, -2.6373, -2.3291),
            (-0.15, 0.2, 16.67, 1.8732, -2.6558, -1.9290),
            (-0.10, -0.1, -12.57, 1.2426, -2.6870, -1.5642),
            (0.10, 0.0, 0.0, -1.2346, -2.7284, -0.6451),
            (0.20, -0.3, 18.69, -2.5051, -2.6373, -0.4293)]:
        K_test = scalar_lqr(A, Bphi, Q, R)
        tag = f'v={v:+.2f} w={w:+.2f}'
        check(tag + ' K', None if K_test is None else K_test, K)
    print("\t---------------------------")

    Q = 2
    R = 5
    print("\n\tFor Q = 2, R = 5.")
    for v, w, phi_deg, A, Bphi, K in [
            (-0.20, 0.0, 0.0, 2.4691, -2.7284, -2.009),
            (-0.20, 0.3, 18.69, 2.5051, -2.6373, -2.091),
            (-0.15, 0.2, 16.67, 1.8732, -2.6558, -1.6527),
            (-0.10, -0.1, -12.57, 1.2426, -2.6870, -1.2459),
            (0.10, 0.0, 0.0, -1.2346, -2.7284, -0.3252),
            (0.20, -0.3, 18.69, -2.5051, -2.6373, -0.1913)]:
        K_test = scalar_lqr(A, Bphi, Q, R)
        tag = f'v={v:+.2f} w={w:+.2f}'
        check(tag + ' K', None if K_test is None else K_test, K)
    print("\t---------------------------")


    print('\n\n\n TEST 4: build_table()')
    gain_table = build_table(1.0, 1.0)
    check('filled with values', np.count_nonzero(gain_table), (len(SPEED_LEVELS)-1)*len(TURN_LEVELS), 0)
    stable_linearizations = 0 
    for i, v_eq in enumerate(SPEED_LEVELS):
        for j, omega_eq in enumerate(TURN_LEVELS):
            phi_eq, A, Bphi = linearize(v_eq, omega_eq)
            if phi_eq is None:
                Acl = A
                stable_linearizations += 1
            else: 
                Acl = (A - Bphi * gain_table[i,j])
                if Acl < 0: 
                    stable_linearizations += 1
    check_true('Acl = (A - B*K < 0) when v_eq =/= 0', stable_linearizations == len(SPEED_LEVELS)*len(TURN_LEVELS))
    mirror = True
    for i, v_eq in enumerate(SPEED_LEVELS):
        for j, omega_eq in enumerate(TURN_LEVELS):
            if not np.isclose(gain_table[i,j], gain_table[i, len(TURN_LEVELS)-1-j]):
                mirror = False
    check_true('(v, omega) and (v, -omega) have the same K', mirror)
    print("\t---------------------------")

    print('\n\n\n TEST 5: gain_scheduled_controller()')
    for v, w, phi, expected in [(-0.2, 0.0, 0.10, 0.2254), (-0.2, 0.3, 0.40, 0.4719),
                                (-0.1, -0.1, -0.30, -0.2261), (0.2, 0.0, 0.10, 0.0444),
                                (0.2, -0.3, 0.0, -0.4400), 
                                (0.0, 0.0, 0.0, 0.0)]:
        u = gain_scheduled_controller(gain_table, v, w, phi)
        check(f'v={v:+.2f} w={w:+.2f} phi={phi:+.2f}', gain_scheduled_controller(gain_table, v, w, phi), expected)
    print("\t---------------------------")

    # # print('\n\n\n TEST 6: snap_to_level() and command_to_send()')
    # # check_true('snap: 0.2000001 finds 0.2', snap_to_level(SPEED_LEVELS, 0.2000001) == 0.2)
    # # check_true('snap: -0.3 finds -0.3', snap_to_level(TURN_LEVELS, -0.3) == -0.3)
    # # check_true('snap: 0.15 is not a level', snap_to_level(SPEED_LEVELS, 0.15) is None)

    # out = command_to_send(gain_table, -0.2, 0.0, 0.10)
    # check('reverse in table: omega_out', None if out is None else out[0], 0.2254)
    # out = command_to_send(gain_table, -0.2000001, 0.3, 0.40)
    # check('reverse, command 1e-7 off the level: omega_out', None if out is None else out[0], 0.4719)
    # check('reverse in table: phi_eq [deg]',
    #     None if out is None or out[1] is None else math.degrees(out[1]), 18.69, 0.01)
    # out = command_to_send(gain_table, -0.2, 0.3, 2.0)
    # check('big error is clamped to MAX_ANGULAR_RAD_S', None if out is None else out[0], MAX_ANGULAR_RAD_S)
    # out = command_to_send(gain_table, -0.2, 0.3, -2.0)
    # check('big error is clamped to -MAX_ANGULAR_RAD_S', None if out is None else out[0], -MAX_ANGULAR_RAD_S)

    # check_true('reverse, speed not in table -> refuse (None)', command_to_send(gain_table, -0.15, 0.0, 0.0) is None)
    # check_true('reverse, turn not in table -> refuse (None)', command_to_send(gain_table, -0.2, 0.25, 0.0) is None)
    # check_true('reverse, steady angle over the limit -> refuse (None)',
    #         command_to_send(gain_table, -0.1, 0.5, 0.0) is None)

    # out = command_to_send(gain_table, 0.2, -0.3, 0.0)
    # check('forward in table: omega_out', None if out is None else out[0], -0.44)
    # out = command_to_send(gain_table, 0.15, 0.2, 0.0)
    # check_true('forward, not in table -> passed through unchanged', out == (0.2, None))
    # out = command_to_send(gain_table, 0.1, 0.3, 0.0)
    # check_true('forward, steady angle over the limit -> passed through unchanged', out == (0.3, None))
    # print("\t---------------------------")

    if failures:
        print("\n\n\n\t" +  f'{len(failures)} FAILED. Scroll up to see which tests were failed.' )
        print("\t---------------------------")
        return 1
    print("\n\n\t" + 'All tests passed. (Yay!)')
    print("\t---------------------------")
    return 0


def main(args=None):
    # `trailer_lqr test` checks the maths and exits; with no argument the node runs.
    if 'test' in (sys.argv[1:] if args is None else args):
        sys.exit(run_tests())

    # Keep the ROS context alive on Ctrl-C so the final zero /cmd_vel can be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('trailer_lqr')

    # Built once here; takes well under a millisecond.
    gain_table = build_table(Q, R)

    # What the callbacks share.
    state = {
        'v': 0.0, 'w': 0.0,             # the held teleop command
        'trailer': None,                # latest TrailerAngle
        'trailer_time': None,           # when it arrived
        'sending': False,               # True while we are publishing a motion
    }

    cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)
    debug_pub = node.create_publisher(Vector3Stamped, '~/debug', 10)

    def publish(v, w):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        cmd_pub.publish(cmd)

    def stop(reason):
        """Zero velocity, and forget the held command: press a key again to restart."""
        state['v'] = 0.0
        state['w'] = 0.0
        publish(0.0, 0.0)
        node.get_logger().warn(f'STOPPED: {reason} Press a key to drive again.')
        state['sending'] = False

    def on_teleop(msg):
        state['v'] = msg.linear.x
        state['w'] = msg.angular.z

    def on_trailer(msg):
        state['trailer'] = msg
        state['trailer_time'] = node.get_clock().now()

    def trailer_angle():
        """The trailer angle [rad], or None (after stopping) if it cannot be trusted."""
        trailer = state['trailer']
        if trailer is None:
            stop('no /trailer_angle yet (is trailer_sensor running?).')
            return None

        age_s = (node.get_clock().now() - state['trailer_time']).nanoseconds * 1e-9
        if age_s > TRAILER_TIMEOUT_S:
            stop(f'no /trailer_angle for {age_s:.2f} s.')
            return None

        if not trailer.magnet_ok:
            stop('the trailer sensor magnet is not OK, so the angle is not reliable.')
            return None

        phi = trailer.angle_rad
        if abs(phi) > math.radians(MAX_TRAILER_ANGLE_DEG):
            stop(f'trailer angle {math.degrees(phi):.1f} deg '
                f'(limit {MAX_TRAILER_ANGLE_DEG:.0f} deg). Jackknife risk.')
            return None
        return phi

    def on_timer():
        v_cmd, w_cmd = state['v'], state['w']

        # Teleop says stop: send ONE zero (so it takes effect), then stay quiet. Staying quiet
        # matters: any /cmd_vel would take the wheels out of the teleop's coast or brake.
        if v_cmd == 0.0 and w_cmd == 0.0:
            if state['sending']:
                publish(0.0, 0.0)
                state['sending'] = False
            return

        phi = trailer_angle()
        if phi is None:
            return

        result = command_to_send(gain_table, v_cmd, w_cmd, phi)
        if result is None:
            stop('reversing with a command that is not in the gain table '
                f'(v={v_cmd:+.2f}, w={w_cmd:+.2f}).')
            return

        w_out, phi_eq = result
        publish(v_cmd, w_out)
        state['sending'] = True

        if phi_eq is not None:
            debug = Vector3Stamped()
            debug.header.stamp = node.get_clock().now().to_msg()
            debug.vector.x = float(phi)
            debug.vector.y = float(phi_eq)
            debug.vector.z = float(w_out)
            debug_pub.publish(debug)

    node.create_subscription(Twist, 'cmd_vel_teleop', on_teleop, 10)
    trailer_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(TrailerAngle, 'trailer_angle', on_trailer, trailer_qos)
    node.create_timer(1.0 / RATE_HZ, on_timer)

    node.get_logger().info(
        f'trailer_lqr ready: {np.count_nonzero(gain_table)} gains, '
        f'L={L:.3f} m, a={a:.3f} m, '
        f'Q={Q:g}, R={R:g}. Waiting for /cmd_vel_teleop.')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if state['sending']:
            for _ in range(3):
                publish(0.0, 0.0)
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
