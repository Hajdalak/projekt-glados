from __future__ import print_function
import cv2
import numpy as np
from robolab_turtlebot import Turtlebot, Rate


def nothing(x):
    pass


def create_trackbars():
    cv2.namedWindow("Trackbars")

    cv2.createTrackbar("H min", "Trackbars", 20, 179, nothing)
    cv2.createTrackbar("H max", "Trackbars", 38, 179, nothing)
    cv2.createTrackbar("S min", "Trackbars", 120, 255, nothing)
    cv2.createTrackbar("S max", "Trackbars", 255, 255, nothing)
    cv2.createTrackbar("V min", "Trackbars", 120, 255, nothing)
    cv2.createTrackbar("V max", "Trackbars", 255, 255, nothing)

    cv2.createTrackbar("Blur", "Trackbars", 1, 20, nothing)
    cv2.createTrackbar("Erode", "Trackbars", 0, 10, nothing)
    cv2.createTrackbar("Dilate", "Trackbars", 0, 10, nothing)

    cv2.createTrackbar("Band y1", "Trackbars", 180, 479, nothing)
    cv2.createTrackbar("Band y2", "Trackbars", 260, 479, nothing)


def get_trackbar_values():
    h_min = cv2.getTrackbarPos("H min", "Trackbars")
    h_max = cv2.getTrackbarPos("H max", "Trackbars")
    s_min = cv2.getTrackbarPos("S min", "Trackbars")
    s_max = cv2.getTrackbarPos("S max", "Trackbars")
    v_min = cv2.getTrackbarPos("V min", "Trackbars")
    v_max = cv2.getTrackbarPos("V max", "Trackbars")

    blur = cv2.getTrackbarPos("Blur", "Trackbars")
    erode_iter = cv2.getTrackbarPos("Erode", "Trackbars")
    dilate_iter = cv2.getTrackbarPos("Dilate", "Trackbars")

    y1 = cv2.getTrackbarPos("Band y1", "Trackbars")
    y2 = cv2.getTrackbarPos("Band y2", "Trackbars")

    return h_min, h_max, s_min, s_max, v_min, v_max, blur, erode_iter, dilate_iter, y1, y2


def main():
    turtle = Turtlebot(rgb=True)
    rate = Rate(10)

    create_trackbars()

    print("HSV tuner spuštěn.")
    print("Klávesy:")
    print("  q ... konec")
    print("  p ... vypiš aktuální HSV rozsah do terminálu")

    while not turtle.is_shutting_down():
        frame = turtle.get_rgb_image()

        if frame is None:
            print("RGB frame is None.")
            rate.sleep()
            continue

        # Pokud kamera vrací BGR, tohle je správně.
        # Pokud by barvy vypadaly divně, zkus místo COLOR_BGR2HSV dát COLOR_RGB2HSV.
        h_min, h_max, s_min, s_max, v_min, v_max, blur, erode_iter, dilate_iter, y1, y2 = get_trackbar_values()

        if y1 > y2:
            y1, y2 = y2, y1

        proc = frame.copy()

        if blur > 0:
            k = 2 * blur + 1
            proc = cv2.GaussianBlur(proc, (k, k), 0)

        hsv = cv2.cvtColor(proc, cv2.COLOR_BGR2HSV)

        lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper = np.array([h_max, s_max, v_max], dtype=np.uint8)

        mask = cv2.inRange(hsv, lower, upper)

        if erode_iter > 0:
            mask = cv2.erode(mask, None, iterations=erode_iter)

        if dilate_iter > 0:
            mask = cv2.dilate(mask, None, iterations=dilate_iter)

        result = cv2.bitwise_and(frame, frame, mask=mask)

        # zvýraznění pásu, ve kterém chceš hledat otvor garáže
        preview = frame.copy()
        cv2.line(preview, (0, y1), (preview.shape[1] - 1, y1), (0, 255, 255), 2)
        cv2.line(preview, (0, y2), (preview.shape[1] - 1, y2), (0, 255, 255), 2)

        band_mask = mask[y1:y2 + 1, :]
        if band_mask.size > 0:
            col_sum = np.sum(band_mask > 0, axis=0)
            yellow_cols = col_sum > max(1, (y2 - y1 + 1) // 8)

            # najdi nejdelší mezeru bez žluté
            best_start = None
            best_end = None
            cur_start = None

            for x in range(len(yellow_cols)):
                if not yellow_cols[x]:
                    if cur_start is None:
                        cur_start = x
                else:
                    if cur_start is not None:
                        cur_end = x - 1
                        if best_start is None or (cur_end - cur_start) > (best_end - best_start):
                            best_start = cur_start
                            best_end = cur_end
                        cur_start = None

            if cur_start is not None:
                cur_end = len(yellow_cols) - 1
                if best_start is None or (cur_end - cur_start) > (best_end - best_start):
                    best_start = cur_start
                    best_end = cur_end

            if best_start is not None and best_end is not None:
                opening_center_x = (best_start + best_end) // 2

                cv2.line(preview, (best_start, y1), (best_start, y2), (0, 0, 255), 2)
                cv2.line(preview, (best_end, y1), (best_end, y2), (0, 0, 255), 2)
                cv2.line(preview, (opening_center_x, y1), (opening_center_x, y2), (255, 0, 0), 2)

                cv2.putText(
                    preview,
                    "opening_center_x = {}".format(opening_center_x),
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA
                )

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        top = np.hstack((preview, mask_bgr))
        bottom = np.hstack((result, frame))
        combined = np.vstack((top, bottom))

        cv2.imshow("HSV garage tuner", combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        if key == ord('p'):
            print("----- AKTUALNI NASTAVENI -----")
            print("GARAGE_MIN_H = {}".format(h_min))
            print("GARAGE_MAX_H = {}".format(h_max))
            print("GARAGE_MIN_S = {}".format(s_min))
            print("GARAGE_MAX_S = {}".format(s_max))
            print("GARAGE_MIN_V = {}".format(v_min))
            print("GARAGE_MAX_V = {}".format(v_max))
            print("Band y1 = {}".format(y1))
            print("Band y2 = {}".format(y2))
            print("------------------------------")

        rate.sleep()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()