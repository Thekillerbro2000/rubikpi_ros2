from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # motor_control + the HW1 route (waypoint_follower)
    motor_node = Node(
        package='robot_control',
        executable='motor_control',
        name='motor_controller_node',
        output='screen',
        emulate_tty=True,
    )

    controller_node = Node(
        package='robot_control',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        emulate_tty=True,
    )

    # When the waypoint run finishes (or fails), bring the whole system down.
    shutdown_when_controller_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_node,
            on_exit=[Shutdown(reason='Waypoint run finished')]
        )
    )

    # If the motor controller exits for any reason, also bring everything down.
    shutdown_when_motor_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=motor_node,
            on_exit=[Shutdown(reason='Motor controller exited')]
        )
    )

    return LaunchDescription([
        motor_node,
        controller_node,
        shutdown_when_controller_exits,
        shutdown_when_motor_exits,
    ])
