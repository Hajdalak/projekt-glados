from __future__ import print_function

# Role of this module:
# - Show the live robot RGB image together with HSV-based masks.
# - Tune HSV and shape filtering parameters interactively.

import cv2
import numpy as np
from robolab_turtlebot import Rate, Turtlebot

WINDOW_MAIN = 'Turtlebot HSV tuner'
WINDOW_MASK = 'Raw HSV mask'
WINDOW_FILTERED = 'Filtered mask (area + axis ratio)'


DEFAULTS = {
	'min_h': 40,
	'max_h': 86,
	'min_s': 80,
	'max_s': 255,
	'min_v': 35,
	'max_v': 255,
	'min_area': 700,
	'max_area': 6000,
	'axis_ratio_x100': 75,
	'kernel_size': 5,
	'morph_on': 1,
}


def _noop(_):
	pass


def _make_trackbars():
	cv2.namedWindow(WINDOW_MAIN, cv2.WINDOW_NORMAL)
	cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)
	cv2.namedWindow(WINDOW_FILTERED, cv2.WINDOW_NORMAL)

	cv2.createTrackbar('min_h', WINDOW_MAIN, DEFAULTS['min_h'], 179, _noop)
	cv2.createTrackbar('max_h', WINDOW_MAIN, DEFAULTS['max_h'], 179, _noop)
	cv2.createTrackbar('min_s', WINDOW_MAIN, DEFAULTS['min_s'], 255, _noop)
	cv2.createTrackbar('max_s', WINDOW_MAIN, DEFAULTS['max_s'], 255, _noop)
	cv2.createTrackbar('min_v', WINDOW_MAIN, DEFAULTS['min_v'], 255, _noop)
	cv2.createTrackbar('max_v', WINDOW_MAIN, DEFAULTS['max_v'], 255, _noop)

	cv2.createTrackbar('min_area', WINDOW_MAIN, DEFAULTS['min_area'], 30000, _noop)
	cv2.createTrackbar('max_area', WINDOW_MAIN, DEFAULTS['max_area'], 30000, _noop)
	cv2.createTrackbar('axis_ratio_x100', WINDOW_MAIN, DEFAULTS['axis_ratio_x100'], 100, _noop)

	cv2.createTrackbar('kernel_size', WINDOW_MAIN, DEFAULTS['kernel_size'], 31, _noop)
	cv2.createTrackbar('morph_on', WINDOW_MAIN, DEFAULTS['morph_on'], 1, _noop)


def _read_params():
	params = {
		'min_h': cv2.getTrackbarPos('min_h', WINDOW_MAIN),
		'max_h': cv2.getTrackbarPos('max_h', WINDOW_MAIN),
		'min_s': cv2.getTrackbarPos('min_s', WINDOW_MAIN),
		'max_s': cv2.getTrackbarPos('max_s', WINDOW_MAIN),
		'min_v': cv2.getTrackbarPos('min_v', WINDOW_MAIN),
		'max_v': cv2.getTrackbarPos('max_v', WINDOW_MAIN),
		'min_area': cv2.getTrackbarPos('min_area', WINDOW_MAIN),
		'max_area': cv2.getTrackbarPos('max_area', WINDOW_MAIN),
		'axis_ratio_x100': cv2.getTrackbarPos('axis_ratio_x100', WINDOW_MAIN),
		'kernel_size': cv2.getTrackbarPos('kernel_size', WINDOW_MAIN),
		'morph_on': cv2.getTrackbarPos('morph_on', WINDOW_MAIN),
	}

	if params['min_h'] > params['max_h']:
		params['min_h'], params['max_h'] = params['max_h'], params['min_h']
	if params['min_s'] > params['max_s']:
		params['min_s'], params['max_s'] = params['max_s'], params['min_s']
	if params['min_v'] > params['max_v']:
		params['min_v'], params['max_v'] = params['max_v'], params['min_v']
	if params['min_area'] > params['max_area']:
		params['min_area'], params['max_area'] = params['max_area'], params['min_area']

	return params


def _apply_morph(mask, kernel_size, morph_on):
	if not morph_on:
		return mask

	# Keep kernel odd and >= 1.
	kernel_size = max(1, kernel_size)
	if kernel_size % 2 == 0:
		kernel_size += 1

	kernel = np.ones((kernel_size, kernel_size), np.uint8)
	cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
	return cleaned


def _extract_objects(mask, min_area, max_area, axis_ratio_min):
	num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

	selected_centroids = []
	filtered_mask = np.zeros_like(mask)
	selected_boxes = []

	for label in range(1, num_labels):
		area = int(stats[label, cv2.CC_STAT_AREA])
		if not (min_area <= area <= max_area):
			continue

		width = int(stats[label, cv2.CC_STAT_WIDTH])
		height = int(stats[label, cv2.CC_STAT_HEIGHT])
		if width == 0 or height == 0:
			continue

		axis_ratio = min(width, height) / float(max(width, height))
		if axis_ratio < axis_ratio_min:
			continue

		x = int(stats[label, cv2.CC_STAT_LEFT])
		y = int(stats[label, cv2.CC_STAT_TOP])
		w = width
		h = height

		selected_centroids.append(centroids[label])
		selected_boxes.append((x, y, w, h, area, axis_ratio))
		filtered_mask[labels == label] = 255

	return selected_centroids, selected_boxes, filtered_mask


def _draw_overlay(frame, selected_boxes, selected_centroids):
	annotated = frame.copy()

	for i, (box, centroid) in enumerate(zip(selected_boxes, selected_centroids), start=1):
		x, y, w, h, area, axis_ratio = box
		cx, cy = centroid
		center = (int(round(cx)), int(round(cy)))

		cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 200, 0), 2)
		cv2.circle(annotated, center, 10, (0, 255, 255), 2)
		cv2.circle(annotated, center, 3, (0, 0, 255), -1)
		cv2.putText(
			annotated,
			'id {} A={} r={:.2f}'.format(i, area, axis_ratio),
			(x, max(16, y - 8)),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.45,
			(255, 255, 255),
			1,
			cv2.LINE_AA,
		)

	cv2.putText(
		annotated,
		'Objects: {}'.format(len(selected_centroids)),
		(10, 24),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.7,
		(0, 255, 255),
		2,
		cv2.LINE_AA,
	)
	cv2.putText(
		annotated,
		'q=quit  p=print params  r=reset',
		(10, 48),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.5,
		(200, 200, 200),
		1,
		cv2.LINE_AA,
	)

	return annotated


def _print_params(params):
	axis_ratio_min = params['axis_ratio_x100'] / 100.0
	print('\n=== Copy to vision.py ===')
	print('BALL_MIN_H = {}'.format(params['min_h']))
	print('BALL_MAX_H = {}'.format(params['max_h']))
	print('BALL_MIN_S = {}'.format(params['min_s']))
	print('BALL_MAX_S = {}'.format(params['max_s']))
	print('BALL_MIN_V = {}'.format(params['min_v']))
	print('BALL_MAX_V = {}'.format(params['max_v']))
	print('DEFAULT_MIN_AREA = {}'.format(params['min_area']))
	print('DEFAULT_MAX_AREA = {}'.format(params['max_area']))
	print('DEFAULT_AXIS_RATIO_MIN = {:.2f}'.format(axis_ratio_min))
	print('# kernel_size (for morphology): {}'.format(params['kernel_size']))
	print('# morph_on: {}'.format(params['morph_on']))

	print('\n# Function call example')
	print(
		'vision.show_detection_stream(turtle, max_area={}, min_area={}, axis_tolerance={:.2f})'.format(
			params['max_area'], params['min_area'], axis_ratio_min
		)
	)
	print('=========================\n')


def _reset_trackbars():
	for name, value in DEFAULTS.items():
		cv2.setTrackbarPos(name, WINDOW_MAIN, value)


def main():
	turtle = Turtlebot(rgb=True)
	_make_trackbars()
	rate = Rate(20)

	print('Turtlebot HSV tuner started.')
	print("Controls: 'q' quit, 'p' print current params, 'r' reset sliders.")

	while True:
		turtle.wait_for_rgb_image()
		frame = turtle.get_rgb_image()
		if frame is None:
			rate.sleep()
			continue

		params = _read_params()

		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		lower = np.array([params['min_h'], params['min_s'], params['min_v']], dtype=np.uint8)
		upper = np.array([params['max_h'], params['max_s'], params['max_v']], dtype=np.uint8)

		raw_mask = cv2.inRange(hsv, lower, upper).astype(np.uint8)
		mask = _apply_morph(raw_mask, params['kernel_size'], params['morph_on'])

		axis_ratio_min = params['axis_ratio_x100'] / 100.0
		centroids, boxes, filtered_mask = _extract_objects(
			mask,
			params['min_area'],
			params['max_area'],
			axis_ratio_min,
		)

		annotated = _draw_overlay(frame, boxes, centroids)

		cv2.imshow(WINDOW_MAIN, annotated)
		cv2.imshow(WINDOW_MASK, raw_mask)
		cv2.imshow(WINDOW_FILTERED, filtered_mask)

		key = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			break
		if key == ord('p'):
			_print_params(params)
		if key == ord('r'):
			_reset_trackbars()

		rate.sleep()

	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
