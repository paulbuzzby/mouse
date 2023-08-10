# Simple example of reading the MCP3008 analog input channels and printing
# them all out.
# Author: Tony DiCola
# License: Public Domain
import time

# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
from gpiozero import LED

led = LED(25)

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

timePrevious = 0
timeCurrent = 0

ADCChannel = 6

timeStart = time.perf_counter()

# print('Reading MCP3008 values, press Ctrl-C to quit...')
# Print nice channel column headers.
# print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*range(8)))
# print('-' * 57)
# Main program loop.
cycles = 1000
counter = 0
# Execution loop
#print('current perf = ', time.perf_counter())
print('Running code for ', cycles, 'cycles ...')
while counter <= cycles:
    # Read all the ADC channel values in a list.
    timeCurrent =  time.perf_counter()
    led.off()
    ADCvalue = mcp.read_adc(ADCChannel)
    
    led.on()
    
    ADCvalue = mcp.read_adc(ADCChannel)
    led.off()
    
    # Print the ADC values.
    # print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} |'.format(*values))
    
    counter = counter + 1

print('Done.')
timeFinish = time.perf_counter()
print (cycles, ' in ', timeFinish-timeStart )
    
