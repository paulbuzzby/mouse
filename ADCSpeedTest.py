import time
import numpy as np
from utils import plot_line
#from gpiozero import LED, MCP3008

import RPi.GPIO as GPIO
from MCP3008 import MCP3008


ledPin = 25
LeftSensorChannel = 6

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ledPin,GPIO.OUT)

adc = MCP3008()

def LEDoff(pinNumber) :
    GPIO.output(pinNumber,GPIO.LOW)

def LEDon(pinNumber) :
    GPIO.output(pinNumber,GPIO.HIGH)


# Assining some parameters
timeSampleRate = 0.02  # Sampling period for code execution (s) 0.001 = 1 millisecond
tstop = 5  # Total execution time (s)

vref = 3.3  # Reference voltage for MCP3008
# Preallocating output arrays for plotting
timeRecord = []  # Time (s)
leftAmbientRecord = []  

leftRecord = []  


leftFilteredRecord = []


leftCorrectedRecord = []


# First order digital low-pass filter parameters
fc = 2  # Filter cutoff frequency (Hz)
wc = 2*np.pi*fc  # Cutoff frequency (rad/s)
tau = 1/wc  # Filter time constant (s)
c0 = timeSampleRate/(timeSampleRate+tau)  # Digital filter coefficient
c1 = tau/(timeSampleRate+tau)  # Digital filter coefficient

# Initializing filter previous value
LEDoff(ledPin)
leftValueprev = adc.read( channel = LeftSensorChannel )
time.sleep(timeSampleRate)

# Initializing variables and starting main clock
timePrevious = 0
timeCurrent = 0

timeStart = time.perf_counter()

cycles = 1000
counter = 0
LEDoff(ledPin)
# Execution loop
#print('current perf = ', time.perf_counter())
print('Running code for ', cycles, 'cycles ...')
while counter <= cycles:
        
    timeCurrent =  time.perf_counter()
    
    leftAmbientOffValue = adc.read( channel = LeftSensorChannel )
    
    LEDon(ledPin)
    
    leftOnValueCurr = adc.read( channel = LeftSensorChannel )
    LEDoff(ledPin)
    
    # leftValuefiltered = c0*leftOnValueCurr + c1*leftValueprev
 
    # leftRecord.append(leftOnValueCurr)
    
    # leftFilteredRecord.append(leftValuefiltered)
    
    # leftAmbientRecord.append(leftAmbientOffValue)

    # leftCorrectedRecord.append(leftOnValueCurr - leftAmbientOffValue)


    # Updating output arrays
    # timeRecord.append(timeCurrent)
    #loopTimeEnd = time.perf_counter()
    # Updating previous filtered value
    # leftValueprev = leftValuefiltered

    #time.sleep(timeSampleRate - (loopTimeEnd - loopTimeStart))

    # Updating previous time value
    timePrevious = timeCurrent
    counter = counter+1

print('Done.')
timeFinish = time.perf_counter()
print (cycles, ' in ', timeFinish-timeStart )

# Plotting results
#plot_line([timeRecord]*4, [leftRecord, leftFilteredRecord, leftAmbientRecord, leftCorrectedRecord], yname='Sensor Output (V)', legend=['Left Raw', 'Left Filtered', 'Left Ambient','Left Corrected'],figsize=1200)
#plot_line([timeRecord[1::]], [1000*np.diff(timeRecord)], yname='Sampling Period (ms)')