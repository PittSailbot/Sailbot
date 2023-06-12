import RPi.GPIO as GPIO
import random
from time import sleep


readPin = 12
writePin = 13
gpio_pins_to_use = []
pin_values = {}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in gpio_pins_to_use:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, False)
    pin_values[pin] = False

for i in range(1000):
    pin = random.randint(0, len(gpio_pins_to_use))
    pinVal = pin_values[pin]
    GPIO.output(self.step_pin, not pinVal)
    pin_values[pin] = not pinVal
    sleep(.1)

print("reading")
for i in range(5):
    GPIO.wait_for_edge(readPin, GPIO.RISING)
    print("read 1")
    GPIO.wait_for_edge(readPin, GPIO.FALLING)
    print("read 0")

print("writing")
for i in range(5):
    GPIO.output(writePin, False)
    sleep(.25)
    GPIO.output(writePin, True)
    sleep(.25)



print("reading")
for i in range(5):
    GPIO.wait_for_edge(readPin, GPIO.RISING)
    print("read 1")
    GPIO.wait_for_edge(readPin, GPIO.FALLING)
    print("read 0")