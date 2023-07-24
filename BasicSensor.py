print("hello world")

ledPin = 17
counter = 0
from gpiozero import LED
from gpiozero import MCP3008
from time import sleep

led = LED(4)

# while True:
#     led.on()
#     sleep(1)
#     led.off()
#     sleep(1)

led.off()
rightSensor = MCP3008(channel=0)
baseline = MCP3008(channel=7)

scale = 3.3

startingValue = rightSensor.value * scale
baselineValue = baseline.value * scale

led.on()

while True:
    rightSensor = MCP3008(channel=0)
    voltage = rightSensor.value * scale
    print("channel 0 is: ", '{:.2f}'.format(voltage), "Difference from starting is: ", '{:.2f}'.format(voltage - startingValue))
    print("Basleine = ",'{:.2f}'.format(baselineValue))    
    sleep(0.2)
    counter += 1
    if counter == 100 : break 
led.off()
print("finished")