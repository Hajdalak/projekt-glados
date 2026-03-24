from __future__ import print_function

# Role of this module:
# - Wire high-level robot behavior.
# - Register safety callbacks.
# - Coordinate detection and movement modules.

from robolab_turtlebot import Turtlebot

import detection
import movement

killSwitch = 0
turtle = Turtlebot(rgb=True, pc=True)


def is_stop_requested():
    """Return True when the global safety stop (killSwitch) is active."""
    return killSwitch != 0


def bumper_callback(msg):
    """Emergency bumper callback: latch killSwitch and stop robot motion."""
    global killSwitch

    killSwitch = msg.state
    turtle.cmd_velocity(linear=0, angular=0)
    print('bumper {}. Do neceho jsem narazil.'.format(killSwitch))


def main():
    """Robot entrypoint: safety setup, ball find, center and approach."""

    turtle.register_bumper_event_cb(bumper_callback)

    ball_center = detection.find_ball(turtle, stop_requested=is_stop_requested)
    if ball_center is None:
        print("Byl aktivovan killSwitch, koncim.")
        return

    centered = movement.recenter_to_ball(turtle)
    if centered is None:
        print("Micek nelze vystredit, koncim.")
        return

    cx, cy = centered
    objects = [(cx, cy)]

    movement.drive_to_ball(turtle, objects, 0.3, stop_requested=is_stop_requested)


if __name__ == '__main__':
    main()