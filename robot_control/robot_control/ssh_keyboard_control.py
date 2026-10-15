import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray

# arrow keys arrive as escape sequences; map them onto WASD
ARROW_KEYS = {
    b'\x1b[A': b'w', b'\x1b[B': b's', b'\x1b[C': b'd', b'\x1b[D': b'a',
    b'\x1bOA': b'w', b'\x1bOB': b's', b'\x1bOC': b'd', b'\x1bOD': b'a',
}

MAX_COMMAND = 0.5  # motor_control clips commands to [-0.5, 0.5]
MAX_RUN_DURATION = 30.0  # longest timed run, in seconds

# [L, R] sent while each key is held; every entry can be changed at runtime with Enter.
# A terminal can't report two keys held at once, so combined directions get their own keys.
DEFAULT_COMMANDS = {
    'w': [0.2475, 0.3],     # forward (trimmed to go straight)
    's': [-0.2475, -0.3],   # backward
    'a': [-0.5, 0.5],     # spin left
    'd': [0.5, -0.5],     # spin right
    'q': [0.15, 0.35],    # forward-left curve
    'e': [0.35, 0.15],    # forward-right curve
    'z': [-0.15, -0.35],  # backward-left curve
    'c': [-0.35, -0.15],  # backward-right curve
}

KEY_NAMES = {
    'w': 'forward', 's': 'backward', 'a': 'spin left', 'd': 'spin right',
    'q': 'forward-left', 'e': 'forward-right', 'z': 'backward-left', 'c': 'backward-right',
}

HELP = """
========================================
   SSH Keyboard Controller (hold a key)
----------------------------------------
    Q  W  E     W/S: forward/backward (or Up/Down)
    A     D     A/D: spin left/right  (or Left/Right)
    Z  S  C     Q/E/Z/C: curves (forward/backward + left/right)

   Enter     : show the [L, R] table and edit one key,
               type "key L R" then Enter (e.g. "s -0.28 -0.3");
               empty line cancels
   M         : timed run - press M, then a direction key, then type
               the seconds and Enter (empty = default); drives that
               key's [L, R] for that long and stops itself
               (X / Space aborts the run, Esc cancels before it starts)
   Space / X : Stop
   Ctrl-C    : Quit
========================================
Publishing motor commands on topic: motor_commands
"""


class SshKeyboardControllerNode(Node):
    """Keyboard teleop that reads keys from the terminal, so it works over SSH (no X display needed)."""

    def __init__(self):
        super().__init__('ssh_keyboard_controller_node')

        # stop if no key event for this long; covers the delay before the terminal's key repeat starts
        self.hold_timeout = self.declare_parameter('hold_timeout', 0.6).value
        # default length of a timed run started with M (used when no time is typed)
        self.run_duration = self.declare_parameter('run_duration', 5.0).value

        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)

        self.commands = {key: list(value) for key, value in DEFAULT_COMMANDS.items()}
        self.direction = None
        self.last_key_time = 0.0
        self.last_command = None
        self.entry_buffer = None  # None = not typing, str = text typed so far
        self.entry_mode = None  # 'edit' = "key L R" edit, 'run' = seconds for a timed run
        self.waiting_for_run_key = False  # True after M, until a direction key is pressed
        self.run_key = None  # direction chosen for the timed run being set up
        self.timed_run = None  # (key, start_time, duration) while a timed run is active
        self.run_timer = None  # one-shot timer that ends the active timed run

        print(HELP)
        self.get_logger().info(f'Default timed run duration: {self.run_duration:.2f} s')

        # publish every 0.05 s; motor_control stops the motors after 0.15 s without commands
        self.timer = self.create_timer(0.05, self.update)

    def print_table(self):
        lines = ['', 'Current commands (key: L, R):']
        for key, (L, R) in self.commands.items():
            lines.append(f'  {key}  {L:+.4f} {R:+.4f}   {KEY_NAMES[key]}')
        sys.stdout.write('\r\n'.join(lines) + '\r\n')
        sys.stdout.flush()

    def read_keys(self):
        """Return all keys pressed since the last call, without blocking."""
        fd = sys.stdin.fileno()
        data = b''
        while select.select([fd], [], [], 0)[0]:
            chunk = os.read(fd, 64)
            if not chunk:
                break
            data += chunk
        for sequence, key in ARROW_KEYS.items():
            data = data.replace(sequence, key)
        return data.decode('utf-8', errors='ignore')

    def handle_key(self, key, now):
        if key in '\r\n':
            # stop and start typing an edit
            self.direction = None
            self.entry_buffer = ''
            self.entry_mode = 'edit'
            self.print_table()
            sys.stdout.write('Edit "key L R": ')
            sys.stdout.flush()
            return
        key = key.lower()
        if key in self.commands:
            self.direction = key
            self.last_key_time = now
        elif key in ' x':
            self.direction = None
        elif key == 'm':
            self.direction = None
            self.waiting_for_run_key = True
            self.get_logger().info(
                f'Timed run: press a direction key ({"".join(self.commands)}), Esc / X to cancel')

    def handle_run_key(self, key):
        """Key pressed after M: choose the direction, then ask for the time."""
        key = key.lower()
        if key in self.commands:
            self.waiting_for_run_key = False
            self.run_key = key
            self.entry_buffer = ''
            self.entry_mode = 'run'
            sys.stdout.write(
                f'\nSeconds for {key} ({KEY_NAMES[key]}) [Enter = {self.run_duration:g}, Esc cancels]: ')
            sys.stdout.flush()
        elif key in ' x\x1b':
            self.waiting_for_run_key = False
            self.get_logger().info('Timed run cancelled')

    def start_timed_run(self, text, now):
        if text:
            try:
                duration = float(text)
                if not 0 < duration <= MAX_RUN_DURATION:
                    raise ValueError
            except ValueError:
                self.get_logger().warn(
                    f'Expected seconds between 0 and {MAX_RUN_DURATION:g}, got "{text}"; timed run cancelled')
                return
        else:
            duration = self.run_duration
        key = self.run_key
        self.timed_run = (key, now, duration)
        # one-shot timer so the run ends at exactly `duration`, not at the next periodic update
        self.run_timer = self.create_timer(duration, self.end_timed_run)
        L, R = self.commands[key]
        self.get_logger().info(
            f'Timed run started: {key} ({KEY_NAMES[key]}) L={L:.4f}, R={R:.4f} for {duration:.2f} s')

    def end_timed_run(self):
        """One-shot timer callback: stop right away instead of waiting for the next update."""
        if self.timed_run is not None:
            self.finish_timed_run(time.monotonic())
            self.publish_current()

    def handle_running_key(self, key, now):
        """During a timed run only X / Space do anything: abort."""
        if key.lower() in ' x':
            self.finish_timed_run(now, aborted=True)

    def finish_timed_run(self, now, aborted=False):
        key, start, _ = self.timed_run
        self.timed_run = None
        self.direction = None
        if self.run_timer is not None:
            self.run_timer.cancel()
            self.destroy_timer(self.run_timer)
            self.run_timer = None
        status = 'ABORTED after' if aborted else 'done:'
        self.get_logger().info(f'Timed run {status} {key} ({KEY_NAMES[key]}) ran {now - start:.3f} s')

    def handle_entry_key(self, key, now):
        """Collect typed characters (the terminal does not echo them) until Enter."""
        if key in '\r\n':
            text = self.entry_buffer.strip()
            mode = self.entry_mode
            self.entry_buffer = None
            self.entry_mode = None
            sys.stdout.write('\n')
            sys.stdout.flush()
            if mode == 'run':
                self.start_timed_run(text, now)
            elif text:
                self.apply_edit(text)
            else:
                self.get_logger().info('Cancelled, commands unchanged')
        elif key == '\x1b':  # Esc
            mode = self.entry_mode
            self.entry_buffer = None
            self.entry_mode = None
            sys.stdout.write('\n')
            sys.stdout.flush()
            self.get_logger().info('Timed run cancelled' if mode == 'run' else 'Cancelled, commands unchanged')
        elif key in '\x7f\x08':  # backspace
            if self.entry_buffer:
                self.entry_buffer = self.entry_buffer[:-1]
                sys.stdout.write('\b \b')
                sys.stdout.flush()
        elif self.entry_mode == 'run' and key not in '0123456789.':
            pass  # only a number makes sense for the seconds
        elif key.isprintable():
            self.entry_buffer += key
            sys.stdout.write(key)
            sys.stdout.flush()

    def apply_edit(self, text):
        parts = text.replace(',', ' ').split()
        try:
            if len(parts) != 3:
                raise ValueError
            key = parts[0].lower()
            if key not in self.commands:
                raise ValueError
            left, right = float(parts[1]), float(parts[2])
        except ValueError:
            self.get_logger().warn(
                f'Expected "key L R" with key one of {"".join(self.commands)}, got "{text}"; unchanged')
            return
        clipped_left = min(MAX_COMMAND, max(-MAX_COMMAND, left))
        clipped_right = min(MAX_COMMAND, max(-MAX_COMMAND, right))
        if (clipped_left, clipped_right) != (left, right):
            self.get_logger().warn(f'Clipped to [-{MAX_COMMAND}, {MAX_COMMAND}] (motor_control limit)')
        self.commands[key] = [clipped_left, clipped_right]
        self.get_logger().info(
            f'{key} ({KEY_NAMES[key]}): L={clipped_left:.4f}, R={clipped_right:.4f}')

    def current_command(self):
        if self.timed_run is not None:
            return tuple(self.commands[self.timed_run[0]])
        if self.direction is None:
            return 0.0, 0.0
        return tuple(self.commands[self.direction])

    def update(self):
        now = time.monotonic()
        for key in self.read_keys():
            if self.entry_buffer is not None:
                self.handle_entry_key(key, now)
            elif self.timed_run is not None:
                self.handle_running_key(key, now)
            elif self.waiting_for_run_key:
                self.handle_run_key(key)
            else:
                self.handle_key(key, now)

        # key released (no more repeats arriving) -> stop
        if self.direction and now - self.last_key_time > self.hold_timeout:
            self.direction = None

        self.publish_current()

    def publish_current(self):
        L, R = self.current_command()
        msg = Float32MultiArray()
        msg.data = [float(L), float(R)]
        self.publisher.publish(msg)

        # log only when the command changes to reduce console spam
        if (L, R) != self.last_command:
            self.get_logger().info(f'Motor command: L={L:.4f}, R={R:.4f}')
            self.last_command = (L, R)

    def send_stop(self):
        try:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0]
            self.publisher.publish(msg)
        except Exception:
            # context may already be shut down; motor_control's timeout stops the motors anyway
            pass


def main(args=None):
    if not sys.stdin.isatty():
        print('Run this with "ros2 run robot_control ssh_keyboard_control" in an interactive terminal '
              '(it cannot read keys when started from a launch file).')
        return 1

    fd = sys.stdin.fileno()
    old_terminal_settings = termios.tcgetattr(fd)

    # handle Ctrl-C ourselves so the stop command is published before ROS shuts down
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SshKeyboardControllerNode()
    try:
        # cbreak: keys arrive immediately without Enter and are not echoed; Ctrl-C still works
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_terminal_settings)
        node.send_stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
