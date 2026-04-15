from __future__ import print_function

import cv2
import numpy as np

DEBUG_GARAGE = True


def garage_dbg(msg):
    if DEBUG_GARAGE:
        print("[GARAGE VISION] {}".format(msg))

# Ball HSV profile.
BALL_MIN_H = 37
BALL_MAX_H = 74
BALL_MIN_S = 98
BALL_MAX_S = 255
BALL_MIN_V = 25
BALL_MAX_V = 255

# Gate poles HSV profile.
GATE_MIN_H = 120
GATE_MAX_H = 159
GATE_MIN_S = 70
GATE_MAX_S = 171
GATE_MIN_V = 0
GATE_MAX_V = 255

# Yellow garage HSV profile.
# These values are only a starting point and may need tuning in lab lighting.
GARAGE_MIN_H = 20
GARAGE_MAX_H = 100
GARAGE_MIN_S = 100
GARAGE_MAX_S = 255
GARAGE_MIN_V = 70
GARAGE_MAX_V = 255

BALL_DEFAULT_MIN_AREA = 200
BALL_DEFAULT_MAX_AREA = 8000
BALL_DEFAULT_AXIS_RATIO_MIN = 0.75
DEFAULT_AXIS_RATIO_MIN = BALL_DEFAULT_AXIS_RATIO_MIN

GATE_DEFAULT_MIN_AREA = 120
GATE_DEFAULT_MAX_AREA = 20000
GATE_DEFAULT_AXIS_RATIO_MIN = 0.10


def _resolve_detection_profile(target_type):
    if target_type == 'gate':
        return {
            'min_h': GATE_MIN_H,
            'max_h': GATE_MAX_H,
            'min_s': GATE_MIN_S,
            'max_s': GATE_MAX_S,
            'min_v': GATE_MIN_V,
            'max_v': GATE_MAX_V,
            'min_area': GATE_DEFAULT_MIN_AREA,
            'max_area': GATE_DEFAULT_MAX_AREA,
            'axis_tolerance': GATE_DEFAULT_AXIS_RATIO_MIN,
        }

    return {
        'min_h': BALL_MIN_H,
        'max_h': BALL_MAX_H,
        'min_s': BALL_MIN_S,
        'max_s': BALL_MAX_S,
        'min_v': BALL_MIN_V,
        'max_v': BALL_MAX_V,
        'min_area': BALL_DEFAULT_MIN_AREA,
        'max_area': BALL_DEFAULT_MAX_AREA,
        'axis_tolerance': BALL_DEFAULT_AXIS_RATIO_MIN,
    }


def get_bgr(turtle):
    try:
        image = turtle.get_rgb_image()
    except Exception as exc:
        print("Failed to read RGB frame: {}".format(exc))
        return None

    if image is None:
        print("RGB frame is None.")
        return None

    return image


def get_hsv(turtle):
    rgb_image = get_bgr(turtle)
    if rgb_image is None:
        garage_dbg("get_hsv: RGB frame je None")
        return None

    garage_dbg("get_hsv: frame shape = {}".format(rgb_image.shape))

    # Pokud by HSV porad nesedelo, zkus COLOR_BGR2HSV.
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    return hsv


def create_hsv_mask(hsv_image, min_h, max_h, min_s, max_s, min_v, max_v):
    lower = np.array([min_h, min_s, min_v], dtype=np.uint8)
    upper = np.array([max_h, max_s, max_v], dtype=np.uint8)
    return cv2.inRange(hsv_image, lower, upper).astype(np.uint8)


def find_centroids(mask, max_area, min_area, axis_tolerance=DEFAULT_AXIS_RATIO_MIN):
    num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask)

    chosen = []
    for label in range(1, num_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if min_area <= area <= max_area:
            width = stats[label, cv2.CC_STAT_WIDTH]
            height = stats[label, cv2.CC_STAT_HEIGHT]
            axis_ratio = min(width, height) / float(max(width, height))
            if axis_ratio >= axis_tolerance:
                chosen.append(centroids[label])
    return chosen


def detect_objects_by_hsv_and_area(
    turtle,
    max_area=None,
    min_area=None,
    axis_tolerance=None,
    target_type='ball',
):
    hsv = get_hsv(turtle)
    if hsv is None:
        return []

    profile = _resolve_detection_profile(target_type)

    if max_area is None:
        max_area = profile['max_area']
    if min_area is None:
        min_area = profile['min_area']
    if axis_tolerance is None:
        axis_tolerance = profile['axis_tolerance']

    mask = create_hsv_mask(
        hsv,
        profile['min_h'],
        profile['max_h'],
        profile['min_s'],
        profile['max_s'],
        profile['min_v'],
        profile['max_v'],
    )

    return find_centroids(mask, max_area, min_area, axis_tolerance=axis_tolerance)


def get_average_3d_point(turtle, cx, cy, window_size=5):
    print("vision.get_average_3d_point started.")

    try:
        pc = turtle.get_point_cloud()
    except Exception as exc:
        print("Failed to read point cloud: {}".format(exc))
        return None

    print("Point cloud frame received.")

    if pc is None:
        print("Point cloud frame is None.")
        return None

    col = int(round(cx))
    row = int(round(cy))
    h, w, _ = pc.shape

    half_w = window_size // 2
    row_start = max(0, row - half_w)
    row_end = min(h, row + half_w + 1)
    col_start = max(0, col - half_w)
    col_end = min(w, col + half_w + 1)

    window_pc = pc[row_start:row_end, col_start:col_end, :]
    valid_points = window_pc.reshape(-1, 3)

    if np.all(np.isnan(valid_points)):
        print("All values in the selected window are NaN.")
        return None

    avg_point = np.nanmean(valid_points, axis=0)

    print(
        "Ball 3D point: X={:.2f}, Y={:.2f}, Z={:.2f} m".format(
            float(avg_point[0]),
            float(avg_point[1]),
            float(avg_point[2]),
        )
    )
    return avg_point


def get_average_depth(turtle, cx, cy, window_size=7):
    try:
        depth = turtle.get_depth_image()
    except Exception as exc:
        print("Failed to read depth image: {}".format(exc))
        return None

    if depth is None:
        print("Depth image is None.")
        return None

    if len(depth.shape) == 3 and depth.shape[2] == 1:
        depth = depth[:, :, 0]

    col = int(round(cx))
    row = int(round(cy))
    h, w = depth.shape

    half_w = window_size // 2
    row_start = max(0, row - half_w)
    row_end = min(h, row + half_w + 1)
    col_start = max(0, col - half_w)
    col_end = min(w, col + half_w + 1)

    window = depth[row_start:row_end, col_start:col_end]
    valid = window[~np.isnan(window)]
    if valid.size == 0:
        print("All values in selected depth window are NaN.")
        return None

    return float(np.mean(valid))


def get_yellow_mask(turtle):
    hsv = get_hsv(turtle)
    if hsv is None:
        garage_dbg("get_yellow_mask: hsv je None")
        return None

    mask = create_hsv_mask(
        hsv,
        GARAGE_MIN_H,
        GARAGE_MAX_H,
        GARAGE_MIN_S,
        GARAGE_MAX_S,
        GARAGE_MIN_V,
        GARAGE_MAX_V,
    )

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    yellow_pixels = int(np.sum(mask > 0))
    total_pixels = int(mask.shape[0] * mask.shape[1])
    yellow_ratio = 100.0 * yellow_pixels / float(total_pixels)

    garage_dbg(
        "mask: yellow_pixels = {}, ratio = {:.2f}% | HSV=({}, {}, {})-({}, {}, {})".format(
            yellow_pixels,
            yellow_ratio,
            GARAGE_MIN_H, GARAGE_MIN_S, GARAGE_MIN_V,
            GARAGE_MAX_H, GARAGE_MAX_S, GARAGE_MAX_V
        )
    )

    return mask

def find_garage_opening_center(
    turtle,
    row_ratio=0.55,
    min_gap_width=40,
):
    """
    Hleda otvor garaze pres vnitrni hrany leve a prave zlate steny.
    Vraci (gap_center_x, gap_width, scan_row) nebo None.
    """
    mask = get_yellow_mask(turtle)
    if mask is None:
        garage_dbg("find_opening: mask je None")
        return None

    h, w = mask.shape

    row = int(round(h * row_ratio))
    y1 = max(0, row - 35)
    y2 = min(h, row + 35)

    band = mask[y1:y2, :]
    if band.size == 0:
        garage_dbg("find_opening: band je prazdny")
        return None

    col_sum = np.sum(band > 0, axis=0)
    threshold = max(2, int(0.10 * band.shape[0]))
    yellow_cols = col_sum >= threshold

    mid = w // 2
    left_cols = yellow_cols[:mid]
    right_cols = yellow_cols[mid:]

    left_count = int(np.sum(left_cols))
    right_count = int(np.sum(right_cols))

    garage_dbg(
        "find_opening: row_ratio={:.2f}, row={}, band=({}, {}), threshold={}, left_yellow_cols={}, right_yellow_cols={}".format(
            row_ratio, row, y1, y2, threshold, left_count, right_count
        )
    )

    if not np.any(left_cols):
        garage_dbg("find_opening: vlevo nevidim zadnou zloutou stenu")
        return None

    if not np.any(right_cols):
        garage_dbg("find_opening: vpravo nevidim zadnou zloutou stenu")
        return None

    left_idx = np.where(left_cols)[0]
    right_idx = np.where(right_cols)[0] + mid

    left_inner = int(np.max(left_idx))
    right_inner = int(np.min(right_idx))

    gap_width = right_inner - left_inner - 1
    gap_center_x = 0.5 * (left_inner + right_inner)
    scan_row = (y1 + y2) // 2

    garage_dbg(
        "find_opening: left_inner={}, right_inner={}, gap_width={}, gap_center_x={:.1f}".format(
            left_inner, right_inner, gap_width, gap_center_x
        )
    )

    if gap_width < min_gap_width:
        garage_dbg(
            "find_opening: mezera je moc uzka ({} < {}), neberu ji".format(
                gap_width, min_gap_width
            )
        )
        return None

    garage_dbg("find_opening: OTVOR NALEZEN")
    return float(gap_center_x), int(gap_width), int(scan_row)