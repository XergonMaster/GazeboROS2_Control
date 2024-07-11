#!/usr/bin/env python3
import time
from Rosmaster_Lib import Rosmaster

def main():
    bot = Rosmaster(com="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")

    try:
        servo = int(input("Ingresa el servo (1-2): "))

        while True:
            if servo==2:
                posicion = int(input("Ingresa la posición del servo (75-130): "))
                if 75 < posicion < 130:
                    bot.set_pwm_servo(2, posicion)
                    print(f"Servo ajustado a la posición {posicion}")
                else:
                    print("Posición inválida. Debe estar entre 75 y 130.")
            else:
                vel = int(input("ingresa vel: "))
                bot.set_pwm_servo(1, vel)

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nInterrupción por teclado detectada. Saliendo...")

    finally:
        del bot
        print("Objeto 'bot' eliminado. Programa terminado.")

if __name__ == "__main__":
    main()
