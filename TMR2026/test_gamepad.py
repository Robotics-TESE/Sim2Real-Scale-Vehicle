"""
test_gamepad.py — Identificar ejes y botones del control.
Corre: python3 test_gamepad.py
Mueve palancas, presiona gatillos y botones para ver qué número tienen.
Ctrl+C para salir.
"""
import time
import os
os.environ["SDL_VIDEODRIVER"] = "dummy"
os.environ["SDL_AUDIODRIVER"] = "dummy"

import pygame
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No se detectó ningún control. Conéctalo por Bluetooth y vuelve a correr.")
    exit(1)

js = pygame.joystick.Joystick(0)
js.init()
print(f"Control: {js.get_name()}")
print(f"Ejes: {js.get_numaxes()}  |  Botones: {js.get_numbuttons()}")
print("-" * 50)
print("Mueve palancas/gatillos o presiona botones...\n")

last_axes = [0.0] * js.get_numaxes()
last_btns = [0]  * js.get_numbuttons()

try:
    while True:
        pygame.event.pump()

        axes = [js.get_axis(i) for i in range(js.get_numaxes())]
        btns = [js.get_button(i) for i in range(js.get_numbuttons())]

        for i, (a, la) in enumerate(zip(axes, last_axes)):
            if abs(a - la) > 0.08:
                print(f"  EJE  {i:2d} = {a:+.2f}")

        for i, (b, lb) in enumerate(zip(btns, last_btns)):
            if b and not lb:
                print(f"  BTN  {i:2d} PRESIONADO")
            elif not b and lb:
                print(f"  BTN  {i:2d} soltado")

        last_axes = axes
        last_btns = btns
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nListo.")
    pygame.quit()
