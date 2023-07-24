print("hello world")

ledPin = 17
counter = 0
from gpiozero import MCP3008
from time import sleep


# while True:
#     led.on()
#     sleep(1)
#     led.off()
#     sleep(1)

dipSwitch = MCP3008(channel=4)

scale = 3.3

startingValue = dipSwitch.value * scale


while True:
    dipSwitch = MCP3008(channel=4)
    voltage = dipSwitch.value * scale
    #print("channel 4 is: ", '{:.2f}'.format(dipSwitch))
    print("channel 4 RAW is: ", dipSwitch.raw_value)
    sleep(0.2)
    counter += 1
    #if counter == 100 : break 
print("finished")