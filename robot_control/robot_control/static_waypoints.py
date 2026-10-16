"""HW1 waypoint route: turn -> drive -> turn for every waypoint, timed open loop.

The moves come from the waypoints file (waypoints_to_moves.py). Each move publishes its
calibrated [L, R] on /motor_commands for the time the calibration gives, then the robot is
held stopped for settle_time before the next move. Does not use velocity_control.

Start at the first waypoint, facing its heading (0 deg = +x). Turns: left = counter-clockwise,
right = clockwise.

    ros2 run robot_control static_waypoints                           # needs motor_control running
    ros2 run robot_control static_waypoints --ros-args -p dry_run:=true
    ros2 run robot_control static_waypoints --ros-args -p waypoints_file:=/home/ubuntu/other.txt
"""
import math
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray

from robot_control.waypoints_to_moves import read_waypoints, waypoints_to_moves

MAX_COMMAND = 0.5  # motor_control clips commands to [-0.5, 0.5]
PUBLISH_PERIOD = 0.05  # s between repeated commands during a move (motor_control stops after 0.15 s without one)

# ======================= CALIBRATION (edit these after calibrating) =======================
# How far the robot moves when a command is held for t seconds is a straight line:
#   amount = speed * t + offset        ->  each command is held for t = (amount - offset) / speed
#   drive: amount in metres,  speed in m/s,   offset in metres
#   turn : amount in degrees, speed in deg/s, offset in degrees
# Any value can be overridden for one run, e.g. --ros-args -p turn_right_speed:=250.0
DRIVE_COMMAND = [0.28, 0.3]         # [L, R] for straight driving (trimmed to go straight)
TURN_LEFT_COMMAND = [-0.5, 0.5]     # [L, R] for spinning left (counter-clockwise)
TURN_RIGHT_COMMAND = [0.5, -0.5]    # [L, R] for spinning right (clockwise)

DRIVE_SPEED = 0.4401                # m/s    t = 2.272 d + 0.108
DRIVE_OFFSET = -0.04754             # m
TURN_LEFT_SPEED = 336.70            # deg/s  t = 0.002970 deg + 0.100
TURN_LEFT_OFFSET = -33.67           # deg
TURN_RIGHT_SPEED = 336.70           # deg/s  assumed same as left
TURN_RIGHT_OFFSET = -33.67          # deg

SETTLE_TIME = 0.5                   # s fully stopped after every move
START_DELAY = 2.0                   # s before the first move
# ==========================================================================================

# Default waypoints file: config/waypoints.txt as installed with the package (see setup.py).
WAYPOINTS_FILE = os.path.join(get_package_share_directory('robot_control'), 'config', 'waypoints.txt')

# Reference: the moves waypoints_to_moves produces for the HW1 waypoints.
# ('drive', metres, waypoint) or ('turn', from_heading_deg, to_heading_deg, waypoint);
# headings in degrees, 0 = facing +x, counter-clockwise positive.
# MOVES = [
#     ('drive', 1.0, 1),                  # (0,0) -> (1,0)              wp1 (1, 0, 0)
#     ('turn', 0, 90, 2),                 # left 90
#     ('drive', 1.0, 2),                  # -> (1,1), facing 90          wp2 (1, 1, 90)
#     ('turn', 90, 0, 3),                 # right 90
#     ('drive', 1.0, 3),                  # -> (2,1)                     wp3 (2, 1, 0)
#     ('turn', 0, 90, 4),                 # left 90
#     ('drive', 1.0, 4),                  # -> (2,2)
#     ('turn', 90, -90, 4),               # right 180                    wp4 (2, 2, -90)
#     ('turn', -90, -135, 5),             # right 45
#     ('drive', math.sqrt(2), 5),         # -> (1,1)
#     ('turn', -135, -45, 5),             # left 90                      wp5 (1, 1, -45)
#     ('turn', -45, -135, 6),             # right 90
#     ('drive', math.sqrt(2), 6),         # -> (0,0)
#     ('turn', -135, 0, 6),               # left 135                     wp6 (0, 0, 0)
# ]


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def moves_to_steps(moves):
    """Convert moves to (kind, amount, waypoint_index) steps.

    kind 'drive' (amount in metres) or 'turn' (amount in radians, + = left, - = right).
    """
    steps = []
    for move in moves:
        if move[0] == 'drive':
            _, distance, index = move
            steps.append(('drive', distance, index))
        elif move[0] == 'turn':
            _, from_deg, to_deg, index = move
            steps.append(('turn', wrap_angle(math.radians(to_deg) - math.radians(from_deg)), index))
        else:
            raise ValueError(f'unknown move "{move[0]}"')
    return steps


def time_for_amount(speed, offset, amount):
    """Solve amount = speed * t + offset for t (never negative)."""
    return max(0.0, (amount - offset) / speed)


class StaticWaypointsNode(Node):
    def __init__(self):
        super().__init__('static_waypoints')

        self.drive_command = self.declare_parameter('drive_command', DRIVE_COMMAND).value
        self.turn_left_command = self.declare_parameter('turn_left_command', TURN_LEFT_COMMAND).value
        self.turn_right_command = self.declare_parameter('turn_right_command', TURN_RIGHT_COMMAND).value
        self.drive_speed = self.declare_parameter('drive_speed', DRIVE_SPEED).value
        self.drive_offset = self.declare_parameter('drive_offset', DRIVE_OFFSET).value
        self.turn_left_speed = self.declare_parameter('turn_left_speed', TURN_LEFT_SPEED).value
        self.turn_left_offset = self.declare_parameter('turn_left_offset', TURN_LEFT_OFFSET).value
        self.turn_right_speed = self.declare_parameter('turn_right_speed', TURN_RIGHT_SPEED).value
        self.turn_right_offset = self.declare_parameter('turn_right_offset', TURN_RIGHT_OFFSET).value
        self.settle_time = self.declare_parameter('settle_time', SETTLE_TIME).value
        self.start_delay = self.declare_parameter('start_delay', START_DELAY).value
        self.waypoints_file = os.path.expanduser(
            self.declare_parameter('waypoints_file', WAYPOINTS_FILE).value)
        # only print the plan, don't move
        self.dry_run = self.declare_parameter('dry_run', False).value

        for name, command in (('drive_command', self.drive_command),
                              ('turn_left_command', self.turn_left_command),
                              ('turn_right_command', self.turn_right_command)):
            if len(command) != 2 or any(abs(c) > MAX_COMMAND for c in command):
                raise ValueError(f'{name} must be [L, R] within +/-{MAX_COMMAND}, got {command}')
        for name, speed in (('drive_speed', self.drive_speed),
                            ('turn_left_speed', self.turn_left_speed),
                            ('turn_right_speed', self.turn_right_speed)):
            if speed <= 0:
                raise ValueError(f'{name} must be > 0, got {speed}')

        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

        self.get_logger().info(
            f'Calibration: drive {self.drive_speed:.4f} m/s {self.drive_offset:+.4f} m, '
            f'left {self.turn_left_speed:.1f} deg/s {self.turn_left_offset:+.1f} deg, '
            f'right {self.turn_right_speed:.1f} deg/s {self.turn_right_offset:+.1f} deg, '
            f'settle {self.settle_time:.2f} s')

    def step_command_and_duration(self, kind, amount):
        if kind == 'drive':
            return self.drive_command, time_for_amount(self.drive_speed, self.drive_offset, amount)
        degrees = math.degrees(abs(amount))
        if amount > 0:
            return self.turn_left_command, time_for_amount(
                self.turn_left_speed, self.turn_left_offset, degrees)
        return self.turn_right_command, time_for_amount(
            self.turn_right_speed, self.turn_right_offset, degrees)

    def describe_step(self, kind, amount):
        if kind == 'drive':
            return f'drive {amount:.3f} m'
        return f'turn {"left " if amount > 0 else "right"} {math.degrees(abs(amount)):6.1f} deg'

    def log_plan(self, waypoints, steps):
        self.get_logger().info(f'Waypoints from {self.waypoints_file}:')
        for index, (x, y, theta) in enumerate(waypoints):
            label = 'start' if index == 0 else f'wp {index}'
            self.get_logger().info(f'  {label:>5}: x={x:+.2f} y={y:+.2f} heading={theta:+d} deg')
        self.get_logger().info('Plan:')
        total_time = total_drive = total_turn = 0.0
        for kind, amount, index in steps:
            command, duration = self.step_command_and_duration(kind, amount)
            total_time += duration + self.settle_time
            if kind == 'drive':
                total_drive += amount
            else:
                total_turn += abs(amount)
            self.get_logger().info(
                f'  -> wp {index}: {self.describe_step(kind, amount)}   '
                f'L={command[0]:+.3f} R={command[1]:+.3f} for {duration:.3f} s')
        self.get_logger().info(
            f'Total: {total_drive:.2f} m driving, {math.degrees(total_turn):.0f} deg turning, '
            f'about {total_time:.1f} s')

    def publish(self, L, R):
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.publisher.publish(msg)

    def hold(self, L, R, duration):
        """Publish [L, R] repeatedly for duration seconds."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.publish(L, R)
            time.sleep(min(PUBLISH_PERIOD, max(0.0, end - time.monotonic())))

    def stop(self):
        try:
            self.publish(0.0, 0.0)
        except Exception:
            # context may already be shut down; motor_control's timeout stops the motors anyway
            pass

    def run(self):
        waypoints = read_waypoints(self.waypoints_file)  # [(x, y, heading_deg), ...]
        steps = moves_to_steps(waypoints_to_moves(waypoints))
        self.log_plan(waypoints, steps)
        if self.dry_run:
            self.get_logger().info('dry_run is set: not moving')
            return

        self.get_logger().info(f'Starting in {self.start_delay:.1f} s (Ctrl-C to abort)')
        self.hold(0.0, 0.0, self.start_delay)
        start = time.monotonic()
        current_waypoint = 0
        for kind, amount, index in steps:
            if index != current_waypoint:
                x, y, theta = waypoints[index]
                self.get_logger().info(f'Heading to waypoint {index}: ({x}, {y}, {theta} deg)')
                current_waypoint = index
            command, duration = self.step_command_and_duration(kind, amount)
            self.get_logger().info(f'  {self.describe_step(kind, amount)} ({duration:.3f} s)')
            self.hold(command[0], command[1], duration)
            self.hold(0.0, 0.0, self.settle_time)
        self.get_logger().info(f'Done: reached waypoint {current_waypoint} in {time.monotonic() - start:.1f} s')


def main(args=None):
    # handle Ctrl-C ourselves so the stop command is published before ROS shuts down
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = StaticWaypointsNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted, stopping')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
