from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate, get_time

def odometry(turtle):
    turtle.wait_for_odometry()
    print('Odometry: {}'.format(turtle.get_odometry()))

def rgbImage(turtle):
    turtle = Turtlebot(rgb=True)
    turtle.wait_for_rgb_image()
    rgb = turtle.get_rgb_image()
    print('RGB shape: {}'.format(rgb.shape))
    print('RGB at 20,20: {}'.format(rgb[20, 20, :]))

def depthImage(turtle):
    turtle = Turtlebot(depth=True)
    turtle.wait_for_depth_image()
    depth = turtle.get_depth_image()
    print('Depth shape: {}'.format(depth.shape))
    print('Depth at 20,20: {}'.format(depth[20, 20]))

def pointCloud(turtle):
    turtle = Turtlebot(pc=True)
    turtle.wait_for_point_cloud()
    pc = turtle.get_point_cloud()
    print('PointCloud shape: {}'.format(pc.shape))
    print('Point at 20,20: {}'.format(pc[20, 20, :]))
