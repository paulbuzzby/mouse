from gpiozero import PWMOutputDevice
from gpiozero import DigitalOutputDevice
from gpiozero import PhaseEnableMotor
from time import sleep

PWM_PIN_MOT1 = 12
DIR_PIN_MOT1 = 27

motor1 = PhaseEnableMotor (DIR_PIN_MOT1,PWM_PIN_MOT1,True)


def RotateMotorCW():
        print ('Forward')
        motor1.forward(0.5)

def RotateMotorCCW():
        print ('Backwards')
        motor1.backward(0.25)

def StopMotor():
        motor1.stop()

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
                  