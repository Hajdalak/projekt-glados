from __future__ import print_function

# Role of this module:
# - Provide a standalone RGB camera preview tool.
# - Show what the robot currently sees.

import cv2
from robolab_turtlebot import Rate, Turtlebot


def show_camera_stream(turtle):
    """Open a live RGB camera window. Press q to quit."""
    print("Opening live robot camera preview. Press 'q' to quit.")

    rate = Rate(15)
    while True:
        try:
            turtle.wait_for_rgb_image()
            rgb = turtle.get_rgb_image()
        except Exception as exc:
            print("Failed to read RGB frame: {}".format(exc))
            break

        if rgb is not None:
            try:
                cv2.imshow("Robot RGB camera", rgb)
            except cv2.error as exc:
                print("Cannot open GUI window: {}".format(exc))
                print("Run this script in an environment with X/GUI access.")
                break

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        rate.sleep()

    cv2.destroyAllWindows()


def main():
    turtle = Turtlebot(rgb=True)
    show_camera_stream(turtle)


if __name__ == '__main__':
    main()
