from __future__ import print_function
import math
from robolab_turtlebot import Turtlebot, Rate

killSwitch = 0
turtle_ref = None


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def stop(turtle):
    turtle.cmd_velocity(0.0, 0.0)


def bumper_callback(msg):
    global killSwitch
    killSwitch = msg.state
    if turtle_ref is not None:
        turtle_ref.cmd_velocity(0.0, 0.0)
    print('bumper {}'.format(killSwitch))


def rotate_by(turtle, rate, delta_rad, w=0.25, tol=0.08):
    _, _, a0 = turtle.get_odometry()
    target = wrap_pi(a0 + delta_rad)

    while not turtle.is_shutting_down() and killSwitch == 0:
        _, _, a = turtle.get_odometry()
        err = wrap_pi(target - a)

        if abs(err) < tol:
            break

        ang = 0.8 * err

        if ang > w:
            ang = w
        elif ang < -w:
            ang = -w

        turtle.cmd_velocity(0.0, ang)
        rate.sleep()

    stop(turtle)

def drive_straight(turtle, rate, dist_m, v=0.18, tol=0.03):
    # KROK: uložit si startovní pozici
    x0, y0, _ = turtle.get_odometry()

    # KROK: jet rovně, dokud neujedeme požadovanou vzdálenost
    while not turtle.is_shutting_down() and killSwitch == 0:
        x, y, _ = turtle.get_odometry()
        d = math.hypot(x - x0, y - y0)

        # KROK: pokud už jsme ujeli dost, skončíme
        if d >= dist_m - tol:
            break

        # KROK: poslat robotovi příkaz na jízdu rovně
        turtle.cmd_velocity(v, 0.0)
        rate.sleep()

    # KROK: po dojetí zastavit
    stop(turtle)


def maneuver_start_face_ball(turtle,
                             side_m=0.30,
                             v=0.18,
                             w=0.6):
    """
    Start:
      - robot ~30 cm od míčku
      - robot míří čelem na míček

    Průběh:
      1) vstupní otočení
      2) hexagonový pohyb kolem míčku
      3) závěrečné dorovnání
    """
    rate = Rate(10)

    # =========================================================
    # KROK 1: VSTUPNÍ OTOČENÍ
    # Robot se na začátku otočí o 60 stupňů.
    # Tady hned poznáš, jestli je správně znaménko rotace.
    # =========================================================
    rotate_by(turtle, rate, delta_rad=-math.radians(60), w=w)

    # KROK 1a: pokud byl aktivován bumper / killswitch, skončíme
    if killSwitch != 0:
        stop(turtle)
        return

    # =========================================================
    # KROK 2: POHYB PO STRANÁCH "HEXAGONU"
    # V každé iteraci:
    #   - robot ujede rovně jednu stranu
    #   - pak se otočí o 60 stupňů
    # Pokud se chyba objeví až tady, problém je spíš v cyklu
    # než v úvodním natočení.
    # =========================================================
    for i in range(6):
        # KROK 2.1: kontrola killswitche před jízdou
        if killSwitch != 0:
            break

        # KROK 2.2: jízda rovně po jedné straně
        drive_straight(turtle, rate, dist_m=side_m, v=v)

        # KROK 2.3: kontrola killswitche po jízdě
        if killSwitch != 0:
            break

        # KROK 2.4: otočení o 60 stupňů pro další stranu
        rotate_by(turtle, rate, delta_rad=+math.radians(60), w=w)

    # KROK 2a: pokud byl aktivován bumper / killswitch, skončíme
    if killSwitch != 0:
        stop(turtle)
        return

    # =========================================================
    # KROK 3: ZÁVĚREČNÉ DOROVNÁNÍ
    # Po dokončení objíždění se robot na konci ještě dorovná.
    # Pokud vše předtím funguje a problém je až tady,
    # je chyba v tomto posledním otočení.
    # =========================================================
    rotate_by(turtle, rate, delta_rad=-math.radians(90), w=w)

    # KROK 4: finální zastavení
    stop(turtle)


def drive_around(turtle):
    global turtle_ref
    turtle_ref = turtle

    print("Začínám s objížděním míčku.")
    
    # KROK A: vynulování odometrie před manévrem
    turtle.reset_odometry()
    wait_rate = Rate(10)

    while True:
        x, y, a = turtle.get_odometry()
        if x == 0 and y == 0 and a == 0:
            break
        wait_rate.sleep()

    # KROK B: registrace bumper callbacku kvůli bezpečnosti
    turtle.register_bumper_event_cb(bumper_callback)

    # KROK C: spuštění celého objížděcího manévru
    maneuver_start_face_ball(turtle, side_m=0.40, v=0.18, w=0.6)