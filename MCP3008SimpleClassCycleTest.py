from MCP3008 import MCP3008
import time

import RPi.GPIO as GPIO

ledPin = 25
ADCChannel = 6

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ledPin,GPIO.OUT)

adc = MCP3008()

def LEDoff(pinNumber) :
    GPIO.output(pinNumber,GPIO.LOW)

def LEDon(pinNumber) :
    GPIO.output(pinNumber,GPIO.HIGH)

timePrevious = 0
timeCurrent = 0



timeStart = time.perf_counter()
adcValues = []
cycles = 1000
counter = 0
LEDoff(ledPin)
# Execution loop
#print('current perf = ', time.perf_counter())
print('Running code for ', cycles, 'cycles ...')
while counter <= cycles:

    
    timeCurrent =  time.perf_counter()
    value = adc.read( channel = ADCChannel ) 
    
    LEDon(ledPin)
    
    value = adc.read( channel = ADCChannel ) 
    adcValues.append(value)
    LEDoff(ledPin)
    
    # Print the ADC values.
    # print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))
    
    counter = counter + 1

print('Done.')
timeFinish = time.perf_counter()
#print(adcValues)
print (cycles, ' in ', timeFinish-timeStart )
#print("Applied voltage: %.2f" % (value / 1023.0 * 3.3) )
