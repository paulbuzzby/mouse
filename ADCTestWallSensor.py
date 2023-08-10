import time
import numpy as np
from utils import plot_line

from MCP3008 import MCP3008
import RPi.GPIO as GPIO

ledPin = 25
ADCChannel = 6

adc = MCP3008()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ledPin,GPIO.OUT)

def LEDoff(pinNumber) :
    GPIO.output(pinNumber,GPIO.LOW)

def LEDon(pinNumber) :
    GPIO.output(pinNumber,GPIO.HIGH)

# Creating ADC channel object
leftSensorChannel = 6
rightSensorChannel = 1

# Assining some parameters
timeSampleRate = 0.02  # Sampling period for code execution (s) 0.001 = 1 millisecond
tstop = 10  # Total execution time (s)

vref = 3.3  # Reference voltage for MCP3008
# Preallocating output arrays for plotting
timeRecord = []  # Time (s)
leftAmbientRecord = []  
rightAmbientRecord = []

leftRecord = []  
rightRecord = []

leftFilteredRecord = []
rightFilteredRecord = []  # Fitlered voltage output value (V)

leftCorrectedRecord = []
rightCorrectedRecord = []

# First order digital low-pass filter parameters
fc = 2  # Filter cutoff frequency (Hz)
wc = 2*np.pi*fc  # Cutoff frequency (rad/s)
tau = 1/wc  # Filter time constant (s)
c0 = timeSampleRate/(timeSampleRate+tau)  # Digital filter coefficient
c1 = tau/(timeSampleRate+tau)  # Digital filter coefficient

# Initializing filter previous value
rightValueprev = adc.read( channel = rightSensorChannel )
leftValueprev = adc.read( channel = leftSensorChannel )
time.sleep(timeSampleRate)

# Initializing variables and starting main clock
timePrevious = 0
timeCurrent = 0
timeStart = time.perf_counter()
loopTimeStart = timeStart
loopTimeEnd = timeStart
LEDoff(ledPin) # start with LED off
# Execution loop
#print('current perf = ', time.perf_counter())
print('Running code for', tstop, 'seconds ...')
while timeCurrent <= tstop:
    # Getting current time (s)
    loopTimeStart = time.perf_counter()
    timeCurrent = loopTimeStart - timeStart
    
    # Doing I/O and computations every `tsample` seconds
    if (np.floor(timeCurrent/timeSampleRate) - np.floor(timePrevious/timeSampleRate)) == 1:
                
        leftAmbientOffValue = adc.read( channel = leftSensorChannel )
        rightAmbientOffValue = adc.read( channel = rightSensorChannel )
        
        LEDon(ledPin)
        
        leftOnValueCurr = adc.read( channel = leftSensorChannel )
        rightOnValueCurr = adc.read( channel = rightSensorChannel )        
        LEDoff(ledPin)
        
        leftValuefiltered = c0*leftOnValueCurr + c1*leftValueprev
        rightValuefiltered = c0*rightOnValueCurr + c1*rightValueprev
        
        leftRecord.append(leftOnValueCurr)
        rightRecord.append(rightOnValueCurr)
        
        leftFilteredRecord.append(leftValuefiltered)
        rightFilteredRecord.append(rightValuefiltered)
        
        leftAmbientRecord.append(leftAmbientOffValue)
        rightAmbientRecord.append(rightAmbientOffValue)

        leftCorrectedRecord.append(leftOnValueCurr - leftAmbientOffValue)
        rightCorrectedRecord.append(rightOnValueCurr - rightAmbientOffValue)


        # Updating output arrays
        timeRecord.append(timeCurrent)
        # Updating previous filtered value
        rightValueprev = rightValuefiltered
        leftValueprev = leftValuefiltered

    #time.sleep(timeSampleRate - (loopTimeEnd - loopTimeStart))

    # Updating previous time value
    timePrevious = timeCurrent

print('Done.')

# Plotting results
plot_line([timeRecord]*4, [leftRecord, leftFilteredRecord, leftAmbientRecord, leftCorrectedRecord], yname='Sensor Output (V)', legend=['Left Raw', 'Left Filtered', 'Left Ambient','Left Corrected'],figsize=1200)
#plot_line([timeRecord]*4, [rightRecord, rightFilteredRecord, rightAmbientRecord, rightCorrectedRecord], yname='Sensor Output (V)', legend=['Right RAW', 'Right Filtered', 'Right Ambient', 'Right Corrected'],figsize=1200)
#plot_line([timeRecord[1::]], [1000*np.diff(timeRecord)], yname='Sampling Period (ms)')