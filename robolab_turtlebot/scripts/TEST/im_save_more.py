#Code made by gemini

from __future__ import print_function
import sys
import time  # Přidán modul pro časování
from robolab_turnlebot import Turtlebot
from imageio import imwrite

turtle = Turtlebot(rgb=True)

# Určení základního názvu (prefixu)
if len(sys.argv) > 1:
    prefix = sys.argv[1]
else:
    prefix = 'capture'

counter = 1  # Naše počítadlo pro číslování fotek

print("Spouštím smyčku focení každých 10 vteřin.")
print("Pro ukončení programu stiskni Ctrl+C v terminálu.")

try:
    while True:
        # Robot počká na nejčerstvější snímek a stáhne ho
        turtle.wait_for_rgb_image()
        rgb = turtle.get_rgb_image()

        # Složíme název souboru: např. "capture" + "1" + ".png"
        filename = '{}{}.png'.format(prefix, counter)

        # Uložíme obrázek
        imwrite(filename, rgb)
        print('Obrázek uložen jako: {}'.format(filename))

        # Zvýšíme počítadlo pro další fotku
        counter += 1
        
        # Uspíme program na 10 vteřin
        time.sleep(10)

except KeyboardInterrupt:
    # Tento blok zachytí stisknutí Ctrl+C a slušně se rozloučí
    print("\nFocení bylo úspěšně ukončeno uživatelem.")