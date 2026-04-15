from __future__ import print_function

# Role of this module:
# - Provide a standalone RGB camera preview tool.
# - Show what the robot currently sees.

import cv2
import os
import time
from robolab_turtlebot import Rate, Turtlebot


DEFAULT_HEADLESS_PRINT_PERIOD_SEC = 1.0


def _is_display_available():
    """Return True when GUI display is available for OpenCV windows."""
    display = os.environ.get("DISPLAY", "").strip()
    if not display:
        return False

    # In containerized Linux sessions DISPLAY can be set but unusable.
    if display.startswith(":"):
        display_num = display[1:].split(".")[0]
        if display_num.isdigit():
            x11_socket = "/tmp/.X11-unix/X{}".format(display_num)
            if not os.path.exists(x11_socket):
                return False

    return True


def show_camera_stream_headless(turtle, print_period_sec=DEFAULT_HEADLESS_PRINT_PERIOD_SEC):
    """Run camera loop without GUI and print periodic frame diagnostics."""
    print("GUI display is not available. Running headless preview (stop: Ctrl+C).")

    last_print = 0.0
    rate = Rate(15)

    try:
        while True:
            try:
                turtle.wait_for_rgb_image()
                rgb = turtle.get_rgb_image()
            except Exception as exc:
                print("Failed to read RGB frame: {}".format(exc))
                break

            now = time.time()
            if rgb is not None and (now - last_print) >= print_period_sec:
                print("RGB frame OK, shape={}".format(rgb.shape))
                last_print = now

            rate.sleep()
    except KeyboardInterrupt:
        print("Headless preview stopped by user.")


def show_camera_stream(turtle):
    """Open a live RGB camera window. Press q to quit."""
    if not _is_display_available():
        return show_camera_stream_headless(turtle)

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
                return show_camera_stream_headless(turtle)

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