from robolab_turtlebot import Turtlebot, Rate, get_time

t = Turtlebot()
r = Rate(10)

# počkej, až bude odometrie dostupná
t.wait_for_odometry()

odo0 = t.get_odometry()
if odo0 is None:
    print("Odometry is None (not ready).")
    exit(1)

a0 = odo0[2]

t0 = get_time()
while get_time() - t0 < 1.0:
    t.cmd_velocity(0.0, +0.4)   # fyzicky doleva
    r.sleep()
t.cmd_velocity(0.0, 0.0)

odo1 = t.get_odometry()
if odo1 is None:
    print("Odometry is None after turning.")
    exit(1)

a1 = odo1[2]
print("a0=", a0, "a1=", a1, "delta=", a1 - a0)