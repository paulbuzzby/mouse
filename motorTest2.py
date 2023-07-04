#    C2Plabs.com 
#
#

from gpiozero import PWMOutputDevice
from gpiozero import DigitalOutputDevice
from gpiozero import PhaseEnableMotor
from time import sleep

# GPIO 6 is used for Generating Software PWM
# GPIO 13 & GPIO 19 are used for Motor control pins as per schematic 


PWM_PIN_MOT1 = 12
DIR_PIN_MOT1 = 27
#IN2_PIN_MOT1 = 19

# PWMOutputDevice takes  BCM_PIN number
# Active High 
# intial value
# PWM Frequency
# and Pin_factory which can be ignored

#pwm_pin_mot1 = PWMOutputDevice (PWM_PIN_MOT1,True, 0, 1200)

# DigitalOutputDevice take 
# Pin Nuumber
# Active High
#Initial Value

#cw_pin_mot1 = DigitalOutputDevice (DIR_PIN_MOT1, True, 0)

#ccw_pin_mot1 = DigitalOutputDevice (IN2_PIN_MOT1, True, 0)

motor1 = PhaseEnableMotor (DIR_PIN_MOT1,PWM_PIN_MOT1,True)


def RotateMotorCW():
        print ('Forward')
        motor1.forward(0.5)
        # cw_pin_mot1.on()
        # pwm_pin_mot1.value = 0.5
        


def RotateMotorCCW():
        print ('Backwards')
        motor1.backward(0.25)
        # cw_pin_mot1.off()
        # pwm_pin_mot1.value = 0.25

def StopMotor():
        motor1.stop()
        # cw_pin_mot1.off()
        # pwm_pin_mot1.value = 0

def main():

        try:
                while True:
                        RotateMotorCW()
                        sleep(4)
                        RotateMotorCCW()
                        sleep(4)
        except KeyboardInterrupt:
                print ('Interrupted')
        StopMotor()

if __name__ == "__main__":
        main()
                  