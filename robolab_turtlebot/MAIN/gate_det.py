# Role of this module:
# - Wire high-level robot behavior.
# - Register safety callbacks.
# - Coordinate detection and movement modules.

from __future__ import print_function
from robolab_turtlebot import Turtlebot

import detection
import movement
import drive_around

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

    gate_center = movement.recenter_between_two_objects(turtle, stop_requested=is_stop_requested)
    if gate_center is None:
        print("Pocatecni centrovani selhalo: brána neni videt. Konec.")
        return

    movement.drive_to_ball(turtle, [], target_distance=0.3, target_type='gate', stop_requested=is_stop_requested)

if __name__ == '__main__':
    main()