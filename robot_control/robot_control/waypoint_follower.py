"""HW1 waypoint follower: turn -> drive -> turn for every waypoint, timed open loop.

The moves come from the waypoints file (waypoints_to_moves.py). For each move the node
publishes its [L, R] command on /motor_commands for the time the calibration gives, then
holds the robot stopped for SETTLE_TIME before the next move.

Start at the first waypoint, facing its heading (0 deg = +x). Turns: left = counter-clockwise,
right = clockwise.

    ros2 run robot_control waypoint_follower                           # needs motor_control running
    ros2 run robot_control waypoint_follower --ros-args -p dry_run:=true
    ros2 run robot_control waypoint_follower --ros-args -p waypoints_file:=/home/ubuntu/other.txt
"""
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray

from robot_control.waypoints_to_moves import read_waypoints, turn_amount, waypoints_to_moves

PUBLISH_PERIOD = 0.05  # s between repeated commands during a move (motor_control stops after 0.15 s without one)

# ======================= CALIBRATION (edit these after calibrating) =======================
# Time to hold a command for a move of size x:   t = a * x + b
#   a = seconds per metre (drive) or per degree (turn)
#   b = extra start-up time every move needs (s)
DRIVE_COMMAND = [0.28, 0.3]         # [L, R] for straight driving (trimmed to go straight)
TURN_LEFT_COMMAND = [-0.5, 0.5]     # [L, R] for spinning left (counter-clockwise)
TURN_RIGHT_COMMAND = [0.5, -0.5]    # [L, R] for spinning right (clockwise)

DRIVE_A = 2.272                     # s per metre
DRIVE_B = 0.108                     # s
TURN_LEFT_A = 0.002970              # s per degree
TURN_LEFT_B = 0.100                 # s
TURN_RIGHT_A = 0.002970             # s per degree (assumed same as left)
TURN_RIGHT_B = 0.100                # s

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


class WaypointFollowerNode(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.waypoints_file = os.path.expanduser(
            self.declare_parameter('waypoints_file', WAYPOINTS_FILE).value)
        # only print the plan, don't move
        self.dry_run = self.declare_parameter('dry_run', False).value

        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

        self.get_logger().info(
            f'Calibration (t = a*x + b): drive a={DRIVE_A} s/m b={DRIVE_B} s, '
            f'left a={TURN_LEFT_A} s/deg b={TURN_LEFT_B} s, '
            f'right a={TURN_RIGHT_A} s/deg b={TURN_RIGHT_B} s, settle {SETTLE_TIME} s')

    def command_and_duration(self, move):
        """[L, R] command for one move and how long to hold it (t = a * x + b)."""
        if move[0] == 'drive':
            distance = move[1]
            return DRIVE_COMMAND, DRIVE_A * distance + DRIVE_B
        degrees = turn_amount(move[1], move[2])  # + = left, - = right
        if degrees > 0:
            return TURN_LEFT_COMMAND, TURN_LEFT_A * degrees + TURN_LEFT_B
        return TURN_RIGHT_COMMAND, TURN_RIGHT_A * -degrees + TURN_RIGHT_B

    def describe(self, move):
        if move[0] == 'drive':
            return f'drive {move[1]:.3f} m'
        degrees = turn_amount(move[1], move[2])
        return f'turn {"left " if degrees > 0 else "right"} {abs(degrees):5.1f} deg'

    def log_plan(self, waypoints, moves):
        self.get_logger().info(f'Waypoints from {self.waypoints_file}:')
        for index, (x, y, theta) in enumerate(waypoints):
            label = 'start' if index == 0 else f'wp {index}'
            self.get_logger().info(f'  {label:>5}: x={x:+.2f} y={y:+.2f} heading={theta:+d} deg')
        self.get_logger().info('Plan:')
        total_time = total_drive = total_turn = 0.0
        for move in moves:
            command, duration = self.command_and_duration(move)
            total_time += duration + SETTLE_TIME
            if move[0] == 'drive':
                total_drive += move[1]
            else:
                total_turn += abs(turn_amount(move[1], move[2]))
            self.get_logger().info(
                f'  -> wp {move[-1]}: {self.describe(move)}   '
                f'L={command[0]:+.3f} R={command[1]:+.3f} for {duration:.3f} s')
        self.get_logger().info(
            f'Total: {total_drive:.2f} m driving, {total_turn:.0f} deg turning, '
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
        moves = waypoints_to_moves(waypoints)
        self.log_plan(waypoints, moves)
        if self.dry_run:
            self.get_logger().info('dry_run is set: not moving')
            return

        self.get_logger().info(f'Starting in {START_DELAY:.1f} s (Ctrl-C to abort)')
        self.hold(0.0, 0.0, START_DELAY)
        start = time.monotonic()
        current_waypoint = 0
        for move in moves:
            index = move[-1]
            if index != current_waypoint:
                x, y, theta = waypoints[index]
                self.get_logger().info(f'Heading to waypoint {index}: ({x}, {y}, {theta} deg)')
                current_waypoint = index
            command, duration = self.command_and_duration(move)
            self.get_logger().info(f'  {self.describe(move)} ({duration:.3f} s)')
            self.hold(command[0], command[1], duration)
            self.hold(0.0, 0.0, SETTLE_TIME)
        self.get_logger().info(f'Done: reached waypoint {current_waypoint} in {time.monotonic() - start:.1f} s')


def main(args=None):
    # handle Ctrl-C ourselves so the stop command is published before ROS shuts down
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WaypointFollowerNode()
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
