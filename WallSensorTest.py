print("hello world")

ledPin = 25
counter = 0
from gpiozero import LED
from gpiozero import MCP3008
from time import sleep

led = LED(ledPin)

# while True:
#     led.on()
#     sleep(1)
#     led.off()
#     sleep(1)

led.off()
leftSensor = MCP3008(channel=6)
#rightSensor = MCP3008(channel=1)

scale = 3.3

while True:
    led.off()
    #rightOffValue = rightSensor.value
    leftOffValue = leftSensor.value

    led.on()
    #rightOnValue = rightSensor.value
    leftOnValue = leftSensor.value
    led.off()
    #voltage = rightSensor.value * scale

    #print("RightOff is: ", '{:.2f}'.format(rightOffValue), "Right on is ", '{:.2f}'.format(rightOnValue), " difference is: '{:.2f}'".format(rightOnValue-rightOffValue))
    print("LeftOff is: ", '{:.2f}'.format(leftOffValue), "Left on is ", '{:.2f}'.format(leftOnValue), " difference is: '{:.2f}'".format(leftOnValue-leftOffValue))
    
    sleep(0.02)
    counter += 1
    if counter == 1000 : break 
led.off()
print("finished")