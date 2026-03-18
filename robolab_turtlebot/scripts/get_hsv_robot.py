from __future__ import print_function

import cv2
import numpy as np
from robolab_turtlebot import Turtlebot

# Made by github copilot

WINDOW_MAIN = 'Turtlebot HS tuner'
WINDOW_MASK = 'HS mask'


DEFAULTS = {
	'min_h': 40,
	'max_h': 86,
	'min_s': 80,
	'max_s': 255,
}


def _noop(_):
	pass


def _make_trackbars():
	cv2.namedWindow(WINDOW_MAIN, cv2.WINDOW_NORMAL)
	cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)

	cv2.createTrackbar('min_h', WINDOW_MAIN, DEFAULTS['min_h'], 179, _noop)
	cv2.createTrackbar('max_h', WINDOW_MAIN, DEFAULTS['max_h'], 179, _noop)
	cv2.createTrackbar('min_s', WINDOW_MAIN, DEFAULTS['min_s'], 255, _noop)
	cv2.createTrackbar('max_s', WINDOW_MAIN, DEFAULTS['max_s'], 255, _noop)


def _read_params():
	params = {
		'min_h': cv2.getTrackbarPos('min_h', WINDOW_MAIN),
		'max_h': cv2.getTrackbarPos('max_h', WINDOW_MAIN),
		'min_s': cv2.getTrackbarPos('min_s', WINDOW_MAIN),
		'max_s': cv2.getTrackbarPos('max_s', WINDOW_MAIN),
	}

	if params['min_h'] > params['max_h']:
		params['min_h'], params['max_h'] = params['max_h'], params['min_h']
	if params['min_s'] > params['max_s']:
		params['min_s'], params['max_s'] = params['max_s'], params['min_s']

	return params


def _print_params(params):
	print('\n=== Copy to vision.py ===')
	print('BALL_MIN_H = {}'.format(params['min_h']))
	print('BALL_MAX_H = {}'.format(params['max_h']))
	print('BALL_MIN_S = {}'.format(params['min_s']))
	print('BALL_MAX_S = {}'.format(params['max_s']))
	print('# V is fixed in this lightweight tuner: 0..255')
	print('BALL_MIN_V = 0')
	print('BALL_MAX_V = 255')
	print('=========================\n')


def _reset_trackbars():
	for name, value in DEFAULTS.items():
		cv2.setTrackbarPos(name, WINDOW_MAIN, value)


def main():
	turtle = Turtlebot(rgb=True)
	_make_trackbars()

	print('Turtlebot HS tuner started (lightweight mode).')
	print("Controls: 'q' quit, 'p' print current params, 'r' reset sliders.")

	while True:
		turtle.wait_for_rgb_image()
		frame = turtle.get_rgb_image()
		if frame is None:
			continue

		params = _read_params()

		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		lower = np.array([params['min_h'], params['min_s'], 0], dtype=np.uint8)
		upper = np.array([params['max_h'], params['max_s'], 255], dtype=np.uint8)
		mask = cv2.inRange(hsv, lower, upper).astype(np.uint8)

		highlight = frame
		cv2.putText(
			highlight,
			'q=quit  p=print params  r=reset',
			(10, 24),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.6,
			(255, 255, 255),
			1,
			cv2.LINE_AA,
		)

		cv2.imshow(WINDOW_MAIN, highlight)
		cv2.imshow(WINDOW_MASK, mask)
		cv2.imshow(WINDOW_FILTERED, filtered_mask)

		key = cv2.waitKey(1) & 0xFF
		if key == ord('q'):
			break
		if key == ord('p'):
			_print_params(params)
		if key == ord('r'):
			_reset_trackbars()

	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
