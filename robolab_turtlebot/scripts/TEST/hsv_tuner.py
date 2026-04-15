from __future__ import print_function
import cv2
import numpy as np
import os
from robolab_turtlebot import Turtlebot

# Prázdná funkce, kterou OpenCV vyžaduje pro trackbary
def nic(x):
    pass


def _is_display_available():
    """Return True when an X11 display is available for OpenCV windows."""
    display = os.environ.get("DISPLAY", "").strip()
    if not display:
        return False

    if display.startswith(":"):
        display_num = display[1:].split(".")[0]
        if display_num.isdigit():
            x11_socket = "/tmp/.X11-unix/X{}".format(display_num)
            if not os.path.exists(x11_socket):
                return False

    return True


def _print_display_help():
    """Print practical steps for GUI execution when DISPLAY is unavailable."""
    print("GUI display is not available, so the HSV tuner cannot open OpenCV windows.")
    print("If you want the windows on your computer over SSH, use an X server and connect with 'ssh -Y'.")
    print("If you want the windows on the robot's own monitor, run 'source robolab_turtlebot/scripts/set-display' on the robot first.")
    print("Current DISPLAY='{}'".format(os.environ.get("DISPLAY", "")))

def main():
    print("Spoustim HSV Tuner...")
    print("Prepnuti do oken a stisknuti klavesy 'q' ukonci program.")

    if not _is_display_available():
        _print_display_help()
        return
    
    # Inicializace robota pouze s RGB kamerou
    turtle = Turtlebot(rgb=True)

    # Vytvoření oken
    cv2.namedWindow("Puvodni obraz")
    cv2.namedWindow("Maska")
    cv2.namedWindow("Ovladani", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Ovladani", 400, 300)

    # Vytvoření trackbarů (posuvníků) pro nastavení HSV
    # Hue je v OpenCV v rozsahu 0-179, Saturation a Value 0-255
    cv2.createTrackbar("H Min", "Ovladani", 0, 179, nic)
    cv2.createTrackbar("H Max", "Ovladani", 179, 179, nic)
    cv2.createTrackbar("S Min", "Ovladani", 0, 255, nic)
    cv2.createTrackbar("S Max", "Ovladani", 255, 255, nic)
    cv2.createTrackbar("V Min", "Ovladani", 0, 255, nic)
    cv2.createTrackbar("V Max", "Ovladani", 255, 255, nic)

    while not turtle.is_shutting_down():
        # Získání snímku z kamery robota
        turtle.wait_for_rgb_image()
        frame = turtle.get_rgb_image()

        if frame is None:
            continue

        # Převod z BGR (OpenCV standard) do HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Načtení aktuálních hodnot z posuvníků
        h_min = cv2.getTrackbarPos("H Min", "Ovladani")
        h_max = cv2.getTrackbarPos("H Max", "Ovladani")
        s_min = cv2.getTrackbarPos("S Min", "Ovladani")
        s_max = cv2.getTrackbarPos("S Max", "Ovladani")
        v_min = cv2.getTrackbarPos("V Min", "Ovladani")
        v_max = cv2.getTrackbarPos("V Max", "Ovladani")

        # Vytvoření polí pro minimální a maximální hranici
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])

        # Vytvoření binární masky
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Aplikace masky na původní obraz (jen pro hezčí vizualizaci)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Zobrazení výsledků v oknech
        cv2.imshow("Puvodni obraz", frame)
        cv2.imshow("Maska", mask)
        cv2.imshow("Vysledek po aplikaci masky", result)

        # Čekání 1 ms na stisk klávesy (nutné pro překreslení OpenCV oken)
        # Pokud uživatel stiskne 'q', smyčka se ukončí
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Ukoncuji HSV Tuner.")
            break

    # Bezpečné zavření všech oken po vyskočení ze smyčky
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()