from gpiozero import Button
from time import sleep

button = Button(4)

while True:
    if button.is_pressed:
        print("Button is pressed")
    else:
        print("Button is not pressed")
    sleep(0.2)