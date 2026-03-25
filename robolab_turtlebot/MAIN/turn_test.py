from robolab_turtlebot import Turtlebot, Rate, get_time

t = Turtlebot()
r = Rate(10)

_, _, a0 = t.get_odometry()

t0 = get_time()
while get_time() - t0 < 1.0:
    t.cmd_velocity(0.0, +0.4)   # fyzicky doleva
    r.sleep()
t.cmd_velocity(0.0, 0.0)

_, _, a1 = t.get_odometry()
print("a0=", a0, "a1=", a1, "delta=", a1 - a0)