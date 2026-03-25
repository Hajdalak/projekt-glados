from robolab_turtlebot import Turtlebot, Rate, get_time

t = Turtlebot()
r = Rate(10)

# +0.4 rad/s na 1s
t0 = get_time()
while get_time() - t0 < 1.0:
    t.cmd_velocity(0.0, +0.4)
    r.sleep()
t.cmd_velocity(0.0, 0.0)

# -0.4 rad/s na 1s
t0 = get_time()
while get_time() - t0 < 1.0:
    t.cmd_velocity(0.0, -0.4)
    r.sleep()
t.cmd_velocity(0.0, 0.0)