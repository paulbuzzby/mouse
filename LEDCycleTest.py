# Simple test to cycle turning LED on and off 1000 times and time how long it takes
import time

# Import SPI library (for hardware SPI) and MCP3008 library.
from gpiozero import LED # 1000  in  0.09061666200000218
import RPi.GPIO as GPIO  # 1000  in  0.009964285000023665

ledPin = 25

import pigpio
import time
pi = pigpio.pi()
pi.set_mode(ledPin, pigpio.OUTPUT)


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ledPin,GPIO.OUT)


led = LED(ledPin)

timeCurrent = 0
timeStart = time.perf_counter()

cycles = 1000
counter = 0
led.off() 
print('Running code for ', cycles, 'cycles ...')
while counter <= cycles:
    timeCurrent =  time.perf_counter()
       
    led.on()    
    led.off()   
    counter = counter + 1

print('Done.')
timeFinish = time.perf_counter()
print ('gpiozero did ', cycles, ' in ', timeFinish-timeStart )

timeCurrent = 0
timeStart = time.perf_counter()

cycles = 1000
counter = 0
GPIO.output(ledPin,GPIO.LOW)
while counter <= cycles:
    timeCurrent =  time.perf_counter()
    
    GPIO.output(ledPin,GPIO.HIGH)
    GPIO.output(ledPin,GPIO.LOW)
    counter = counter + 1

timeFinish = time.perf_counter()
print ('RPi.GPIO did ', cycles, ' in ', timeFinish-timeStart )
    

timeCurrent = 0
timeStart = time.perf_counter()

cycles = 1000
counter = 0
pi.write(ledPin, False)
while counter <= cycles:
    timeCurrent =  time.perf_counter()
    
    pi.write(ledPin, True)
    pi.write(ledPin, False)
    counter = counter + 1

timeFinish = time.perf_counter()
print ('PIGPIO did   ', cycles, ' in ', timeFinish-timeStart )

