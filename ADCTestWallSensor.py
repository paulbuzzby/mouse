import time
import numpy as np
from utils import plot_line
from gpiozero import LED, MCP3008


led = LED(25)
# Creating ADC channel object
#sensor = MCP3008(channel=0)
leftSensor = MCP3008(channel=6)
rightSensor = MCP3008(channel=1)

# Assining some parameters
timeSampleRate = 0.02  # Sampling period for code execution (s) 0.001 = 1 millisecond
tstop = 5  # Total execution time (s)

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
led.off()
rightValueprev = rightSensor.value
leftValueprev = leftSensor.value
time.sleep(timeSampleRate)

# Initializing variables and starting main clock
timePrevious = 0
timeCurrent = 0
timeStart = time.perf_counter()
loopTimeStart = timeStart
loopTimeEnd = timeStart

# Execution loop
#print('current perf = ', time.perf_counter())
print('Running code for', tstop, 'seconds ...')
while timeCurrent <= tstop:
    # Getting current time (s)
    loopTimeStart = time.perf_counter()
    timeCurrent = loopTimeStart - timeStart
    
    # Doing I/O and computations every `tsample` seconds
    if (np.floor(timeCurrent/timeSampleRate) - np.floor(timePrevious/timeSampleRate)) == 1:
        
        led.off()
        leftAmbientOffValue = leftSensor.value
        rightAmbientOffValue = rightSensor.value
        
        led.on()
        time.sleep(0.01)
        leftOnValueCurr = leftSensor.value
        rightOnValueCurr = rightSensor.value        
        led.off()
        
        leftValuefiltered = c0*leftOnValueCurr + c1*leftValueprev
        rightValuefiltered = c0*rightOnValueCurr + c1*rightValueprev
        
            
        # Getting potentiometer normalized voltage output
        #sensorValuecurr = sensor.value
        
        # Filtering value
        #valuefiltered = c0*sensorValuecurr + c1*rightValueprev
        
        # Calculating current raw and filtered voltage
        #vcurr = vref*sensorValuecurr
        #vcurrfilt = vref*valuefiltered

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
        #loopTimeEnd = time.perf_counter()
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