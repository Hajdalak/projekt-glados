from __future__ import print_function
import cv2
import numpy as np
from robolab_turtlebot import Turtlebot

# Prázdná funkce, kterou OpenCV vyžaduje pro trackbary
def nic(x):
    pass

def main():
    print("Spoustim HSV Tuner...")
    print("Prepnuti do oken a stisknuti klavesy 'q' ukonci program.")
    
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