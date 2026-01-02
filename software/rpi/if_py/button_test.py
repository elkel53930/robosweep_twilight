from gpiozero import Button
from time import sleep

import sys

# 引数の取得
if len(sys.argv) != 2:
    print("Usage: python3 button_monitor.py <GPIO_PIN>")
    sys.exit(1)
gpio_pin = int(sys.argv[1])
button = Button(gpio_pin)

print(f"Monitoring button on GPIO pin {gpio_pin}... Press Ctrl+C to exit.")

previous_state = None
while True:
    current_state = button.is_pressed
    if current_state != previous_state:
        if current_state:
            print("Button Pressed")
        else:
            print("Button Released")
        previous_state = current_state
    sleep(0.1)