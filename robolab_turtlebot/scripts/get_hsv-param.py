import cv2
import numpy as np
import mss
import tkinter as tk
import threading
import time

try:
    import cv2
except ImportError as exc:
    raise ImportError(
        "Chybí balíček 'opencv-python'. Nainstaluj: pip install opencv-python"
    ) from exc

try:
    import numpy as np
except ImportError as exc:
    raise ImportError(
        "Chybí balíček 'numpy'. Nainstaluj: pip install numpy"
    ) from exc

try:
    import mss
except ImportError as exc:
    raise ImportError(
        "Chybí balíček 'mss'. Nainstaluj: pip install mss"
    ) from exc

try:
    import tkinter as tk
except ImportError as exc:
    raise ImportError(
        "Tkinter není dostupný v aktuálním Pythonu. "
        "Na Windows použij oficiální Python installer (obsahuje tkinter), "
        "na Debian/Ubuntu doinstaluj: sudo apt-get install python3-tk"
    ) from exc

def prepocet_rgb_na_hsv(r, g, b):
    """
    Převede zadanou RGB barvu na HSV hodnoty přizpůsobené pro OpenCV.
    Vrací H, S a V složky, kde H je v rozsahu 0-179, S a V v rozsahu 0-255.
    """
    # Vytvoření 1x1 pixelu (OpenCV vyžaduje pro konverzi 3D pole)
    rgb_pixel = np.uint8([[[r, g, b]]])
    
    # Převod na HSV
    hsv_pixel = cv2.cvtColor(rgb_pixel, cv2.COLOR_RGB2HSV)
    
    # Extrakce jednotlivých složek
    h = int(hsv_pixel[0][0][0])
    s = int(hsv_pixel[0][0][1])
    v = int(hsv_pixel[0][0][2])
    
    return h, s, v

def detekuj_objekt_dle_barvy(frame, lower_green, upper_green, min_w_val=10, max_w_val=2000, min_h_val=10, max_h_val=2000, clean_mask_flag=0, min_area_val=400, max_area_val=100000, use_w=1, use_h_c=1, use_area=1):
    """
    Vyhledá v obraze 'frame' objekty spadající do limitů lower_green a upper_green.
    Vrací obraz s vykreslenými obdélníky a vygenerovanou černobílou masku.
    """
    # Převod do barevného prostoru HSV pro snazší hledání barvy (Hue)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Vytvoření masky - pixely v rozsahu budou bílé, zbytek černý
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Příprava čisté masky pro případ, že je zapnutý filtr šumu
    clean_mask = np.zeros_like(mask)
    
    # Nalezení kontur bílých oblastí na masce
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # Vytvoření obdélníku kolem nalezeného objektu
        x_c, y_c, w_c, h_c = cv2.boundingRect(contour)
        
        area = cv2.contourArea(contour)
        
        # Podmínky pro jednotlivé filtry velikosti
        cond_w = (use_w == 0) or (min_w_val <= w_c <= max_w_val)
        cond_h = (use_h_c == 0) or (min_h_val <= h_c <= max_h_val)
        cond_area = (use_area == 0) or (min_area_val <= area <= max_area_val)
        
        # Filtrování šumu (ignorování příliš malých objektů)
        if cond_w and cond_h and cond_area:
            # Vykreslení obdélníku a textu
            cv2.rectangle(frame, (x_c, y_c), (x_c + w_c, y_c + h_c), (0, 255, 0), 2)
            cv2.putText(frame, "Nalezeny objekt", (x_c, y_c - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Pokud je zapnutý filtr čisté masky, vykreslíme do ní jen tyto platné kontury
            if clean_mask_flag == 1:
                cv2.drawContours(clean_mask, [contour], -1, 255, -1)
                
    # Vrácení vyčištěné nebo původní masky podle stavu checkboxu
    if clean_mask_flag == 1:
        return frame, clean_mask
    else:
        return frame, mask

# Vytvoření hlavního okna (slouží jako hledáček)
root = tk.Tk()
root.title("Sledovaci okno")
root.geometry("500x400+100+100")
root.attributes("-alpha", 0.1)  # Nastaví okno jako téměř průhledné (aby nezkreslovalo barvy pod ním)
root.wm_attributes("-topmost", True) # Udrží okno vždy navrchu

# Vytvoření odděleného okna pro ovládací panel
control_panel = tk.Toplevel(root)
control_panel.title("Ovládací panel")
control_panel.geometry("380x850+620+50")

# Prázdná funkce potřebná pro OpenCV posuvníky
def nic(x):
    pass

# Tkinter proměnné pro posuvníky a checkboxy
var_min_h = tk.IntVar(value=40)
var_max_h = tk.IntVar(value=80)
var_min_s = tk.IntVar(value=50)
var_max_s = tk.IntVar(value=255)
var_min_v = tk.IntVar(value=50)
var_max_v = tk.IntVar(value=255)

var_use_h = tk.IntVar(value=1)
var_use_s = tk.IntVar(value=1)
var_use_v = tk.IntVar(value=1)
var_show_mask = tk.IntVar(value=0)
var_clean_mask = tk.IntVar(value=0)

var_min_w = tk.IntVar(value=10)
var_max_w = tk.IntVar(value=500)
var_min_h_c = tk.IntVar(value=10)
var_max_h_c = tk.IntVar(value=500)
var_min_area = tk.IntVar(value=400)
var_max_area = tk.IntVar(value=100000)

var_use_w = tk.IntVar(value=1)
var_use_h_c = tk.IntVar(value=1)
var_use_area = tk.IntVar(value=1)

def setup_gui():
    """Funkce pro inicializaci Tkinter ovládacího panelu"""
    # H - Hue
    tk.Checkbutton(control_panel, text="Použít H (Odstín)", variable=var_use_h).pack(anchor="w", padx=10, pady=(5, 0))
    tk.Scale(control_panel, from_=0, to=179, orient="horizontal", label="Min H", variable=var_min_h, length=300).pack(padx=10)
    tk.Scale(control_panel, from_=0, to=179, orient="horizontal", label="Max H", variable=var_max_h, length=300).pack(padx=10)
    
    # S - Saturation
    tk.Checkbutton(control_panel, text="Použít S (Sytost)", variable=var_use_s).pack(anchor="w", padx=10, pady=(5, 0))
    tk.Scale(control_panel, from_=0, to=255, orient="horizontal", label="Min S", variable=var_min_s, length=300).pack(padx=10)
    tk.Scale(control_panel, from_=0, to=255, orient="horizontal", label="Max S", variable=var_max_s, length=300).pack(padx=10)
    
    # V - Value
    tk.Checkbutton(control_panel, text="Použít V (Jas)", variable=var_use_v).pack(anchor="w", padx=10, pady=(5, 0))
    tk.Scale(control_panel, from_=0, to=255, orient="horizontal", label="Min V", variable=var_min_v, length=300).pack(padx=10)
    tk.Scale(control_panel, from_=0, to=255, orient="horizontal", label="Max V", variable=var_max_v, length=300).pack(padx=10)

    # Zobrazení a čištění masky
    tk.Label(control_panel, text="Možnosti zobrazení:").pack(anchor="w", padx=10, pady=(10, 0))
    tk.Checkbutton(control_panel, text="Zobrazit černobílou masku", variable=var_show_mask).pack(anchor="w", padx=20)
    tk.Checkbutton(control_panel, text="Vyčistit masku (vykreslit jen platné kontury)", variable=var_clean_mask).pack(anchor="w", padx=20)

    # Rozměry
    tk.Label(control_panel, text="Filtrování velikosti:").pack(anchor="w", padx=10, pady=(10, 0))
    
    tk.Checkbutton(control_panel, text="Filtrovat podle šířky (px)", variable=var_use_w).pack(anchor="w", padx=10)
    frame_w = tk.Frame(control_panel)
    frame_w.pack(fill="x", padx=10)
    tk.Scale(frame_w, from_=0, to=2000, orient="horizontal", label="Min Šířka", variable=var_min_w, length=140).pack(side="left")
    tk.Scale(frame_w, from_=0, to=2000, orient="horizontal", label="Max Šířka", variable=var_max_w, length=140).pack(side="right")

    tk.Checkbutton(control_panel, text="Filtrovat podle výšky (px)", variable=var_use_h_c).pack(anchor="w", padx=10, pady=(5, 0))
    frame_h = tk.Frame(control_panel)
    frame_h.pack(fill="x", padx=10)
    tk.Scale(frame_h, from_=0, to=2000, orient="horizontal", label="Min Výška", variable=var_min_h_c, length=140).pack(side="left")
    tk.Scale(frame_h, from_=0, to=2000, orient="horizontal", label="Max Výška", variable=var_max_h_c, length=140).pack(side="right")
    
    tk.Checkbutton(control_panel, text="Filtrovat podle plochy (px^2)", variable=var_use_area).pack(anchor="w", padx=10, pady=(5, 0))
    frame_area = tk.Frame(control_panel)
    frame_area.pack(fill="x", padx=10)
    tk.Scale(frame_area, from_=0, to=100000, orient="horizontal", label="Min Plocha", variable=var_min_area, length=140).pack(side="left")
    tk.Scale(frame_area, from_=0, to=100000, orient="horizontal", label="Max Plocha", variable=var_max_area, length=140).pack(side="right")

# Spustíme vykreslení panelu
setup_gui()

def process_screen():
    """Hlavní smyčka pro snímání a zpracování obrazu"""
    window_name = "Detekce pod oknem"

    # Inicializace rychlého zachytávání obrazovky
    with mss.mss() as sct:
        while True:
            try:
                # Získání absolutní pozice a rozměrů okna na monitoru
                x = root.winfo_rootx()
                y = root.winfo_rooty()
                w = root.winfo_width()
                h = root.winfo_height()
                
                # Pokud je okno minimalizované, přeskočíme krok
                if w <= 0 or h <= 0:
                    time.sleep(0.1)
                    continue

                # Definice oblasti pod Tkinter oknem
                monitor = {"top": y, "left": x, "width": w, "height": h}
                
                # Zachycení pixelů
                img = np.array(sct.grab(monitor))
                
                # MSS vrací obraz ve formátu BGRA (s alfa kanálem), OpenCV potřebuje BGR
                frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                
                # Načtení aktuálních hodnot z Tkinter UI
                min_h = var_min_h.get()
                max_h = var_max_h.get()
                min_s = var_min_s.get()
                max_s = var_max_s.get()
                min_v = var_min_v.get()
                max_v = var_max_v.get()

                # Načtení stavu "checkboxů" z Tkinteru
                use_h = var_use_h.get()
                use_s = var_use_s.get()
                use_v = var_use_v.get()
                
                show_mask = var_show_mask.get()
                clean_mask_flag = var_clean_mask.get()
                
                min_w_val = var_min_w.get()
                max_w_val = var_max_w.get()
                min_h_val = var_min_h_c.get()
                max_h_val = var_max_h_c.get()
                min_area_val = var_min_area.get()
                max_area_val = var_max_area.get()
                
                use_w_flag = var_use_w.get()
                use_h_c_flag = var_use_h_c.get()
                use_area_flag = var_use_area.get()
                
                # Pokud je checkbox vypnutý (hodnota 0), roztáhneme meze na maximum, čímž se filtrace zruší
                if use_h == 0:
                    min_h, max_h = 0, 179
                if use_s == 0:
                    min_s, max_s = 0, 255
                if use_v == 0:
                    min_v, max_v = 0, 255
                
                # Definice rozsahu pro barvu z posuvníků
                # Původní proměnné zachovány dle instrukcí
                lower_green = np.array([min_h, min_s, min_v])
                upper_green = np.array([max_h, max_s, max_v])
                
                # Volání naší externí detekční funkce
                frame_vykresleno, mask = detekuj_objekt_dle_barvy(
                    frame, lower_green, upper_green, 
                    min_w_val, max_w_val, min_h_val, max_h_val, clean_mask_flag,
                    min_area_val, max_area_val, use_w_flag, use_h_c_flag, use_area_flag
                )
                
                # Zobrazení výsledku v okně OpenCV
                # Pokud je zaškrtnuto "Show Mask", ukážeme masku, jinak ukážeme normální obraz s rámečky
                if show_mask == 1:
                    cv2.imshow(window_name, mask)
                else:
                    cv2.imshow(window_name, frame_vykresleno)
                
                # Ukončení programu po stisknutí klávesy 'q' (musí být aktivní OpenCV okno)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    root.quit()
                    break
            except Exception as e:
                pass
                
    cv2.destroyAllWindows()

# Spuštění detekce ve vedlejším vlákně, aby hlavní GUI okno zůstalo plynulé
thread = threading.Thread(target=process_screen, daemon=True)
thread.start()

# Spuštění hlavní smyčky hledáčku a GUI panelu
root.mainloop()