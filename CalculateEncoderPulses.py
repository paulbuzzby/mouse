import time
import numpy as np
from gpiozero import RotaryEncoder, PhaseEnableMotor
from gpiozero_extended import Motor


PWM_PIN_MOT1 = 12
DIR_PIN_MOT1 = 27

tstop = 5  # Execution duration (s)
tsample = 0.01  # Sampling period (s)
PWMSig = 0.25

#motor1 = PhaseEnableMotor (DIR_PIN_MOT1,PWM_PIN_MOT1,True)
#encoder = RotaryEncoder(18, 17, max_steps=0)

motor1 = Motor(DIR_PIN_MOT1, PWM_PIN_MOT1, 18, 17, 2660)


def main():

    tcurr = 0
    steps = 0
    tstart = time.perf_counter()
    

    # angle = 360 / self._ppr * self._encoder.steps - self._angle0

    ppr = 2660
    startAngle = 0

    try:
        while steps <= ppr:#tcurr <= tstop:
            time.sleep(tsample)

            tcurr = time.perf_counter() - tstart

            #motor1.forward(PWMSig)
            #steps = encoder.steps
            motor1.set_output(PWMSig)
            steps = motor1._encoder.steps
            mAngle = motor1.get_angle()
        

            angle = 360 / ppr * steps - startAngle
            #angle = 360 / self._ppr * self._encoder.steps - self._angle0
            print("encoder count = ", steps, "Angle = ", angle, "Motor1 object angle = ", mAngle)
            

    except KeyboardInterrupt:
            print ('Interrupted')
    motor1.set_output(0)

if __name__ == "__main__":
        main()