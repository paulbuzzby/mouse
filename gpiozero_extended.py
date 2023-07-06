""" 
gpiozero_extended.py contains classes that are not implemented in GPIO Zero
(https://gpiozero.readthedocs.io/en/stable/) or that could use a different
implementation which is more suitable for automation projects.

Author: Eduardo Nigro
    rev 0.0.2
    2021-03-09
"""
import time
from gpiozero import DigitalOutputDevice, PWMOutputDevice, RotaryEncoder, PhaseEnableMotor

import RPi.GPIO as GPIO

import pigpio

class Decoder:

   """Class to decode mechanical rotary encoder pulses."""

   def __init__(self, pi, gpioA, gpioB, callback):

      """
      Instantiate the class with the pi and gpios connected to
      rotary encoder contacts A and B.  The common contact
      should be connected to ground.  The callback is
      called when the rotary encoder is turned.  It takes
      one parameter which is +1 for clockwise and -1 for
      counterclockwise.

      EXAMPLE

      import time
      import pigpio

      import rotary_encoder

      pos = 0

      def callback(way):

         global pos

         pos += way

         print("pos={}".format(pos))

      pi = pigpio.pi()

      decoder = rotary_encoder.decoder(pi, 7, 8, callback)

      time.sleep(300)

      decoder.cancel()

      pi.stop()

      """

      self.pi = pi
      self.gpioA = gpioA
      self.gpioB = gpioB
      self.callback = callback

      self.levA = 0
      self.levB = 0

      self.lastGpio = None

      self.pi.set_mode(gpioA, pigpio.INPUT)
      self.pi.set_mode(gpioB, pigpio.INPUT)

      self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
      self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

      self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
      self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

   def _pulse(self, gpio, level, tick):

      """
      Decode the rotary encoder pulse.

                   +---------+         +---------+      0
                   |         |         |         |
         A         |         |         |         |
                   |         |         |         |
         +---------+         +---------+         +----- 1

             +---------+         +---------+            0
             |         |         |         |
         B   |         |         |         |
             |         |         |         |
         ----+         +---------+         +---------+  1
      """

      if gpio == self.gpioA:
         self.levA = level
      else:
         self.levB = level;

      if gpio != self.lastGpio: # debounce
         self.lastGpio = gpio

         if   gpio == self.gpioA and level == 1:
            if self.levB == 1:
               self.callback(1)
         elif gpio == self.gpioB and level == 1:
            if self.levA == 1:
               self.callback(-1)

   def cancel(self):

      """
      Cancel the rotary encoder decoder.
      """

      self.cbA.cancel()
      self.cbB.cancel()

class Encoder: # https://github.com/nstansby/rpi-rotary-encoder-python/blob/master/encoder.py

    def __init__(self, leftPin, rightPin, callback=None):
        self.leftPin = leftPin
        self.rightPin = rightPin
        self.value = 0
        self.state = '00'
        self.direction = None
        self.callback = callback
        GPIO.setup(self.leftPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.rightPin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.leftPin, GPIO.BOTH, callback=self.transitionOccurred)  
        GPIO.add_event_detect(self.rightPin, GPIO.BOTH, callback=self.transitionOccurred)  

    def transitionOccurred(self, channel):
        p1 = GPIO.input(self.leftPin)
        p2 = GPIO.input(self.rightPin)
        newState = "{}{}".format(p1, p2)

        if self.state == "00": # Resting position
            if newState == "01": # Turned right 1
                self.direction = "R"
            elif newState == "10": # Turned left 1
                self.direction = "L"

        elif self.state == "01": # R1 or L3 position
            if newState == "11": # Turned right 1
                self.direction = "R"
            elif newState == "00": # Turned left 1
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        elif self.state == "10": # R3 or L1
            if newState == "11": # Turned left 1
                self.direction = "L"
            elif newState == "00": # Turned right 1
                if self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)

        else: # self.state == "11"
            if newState == "01": # Turned left 1
                self.direction = "L"
            elif newState == "10": # Turned right 1
                self.direction = "R"
            elif newState == "00": # Skipped an intermediate 01 or 10 state, but if we know direction then a turn is complete
                if self.direction == "L":
                    self.value = self.value - 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                elif self.direction == "R":
                    self.value = self.value + 1
                    if self.callback is not None:
                        self.callback(self.value, self.direction)
                
        self.state = newState

    def getValue(self):
        return self.value


class Motor:
    
    
    def __init__(
        self, pi, direction=None, pwm1=None,
        encoder1=None, encoder2=None, encoderppr=300):
        """
        Class constructor.

        """
        # Identifying motor driver type and assigning appropriate GPIO pins
        if pwm1 and direction:
            self._motor = PhaseEnableMotor(direction, pwm1, True)
        
        else:
            raise Exception('Pin configuration is incorrect.')
        # Checking for encoder
        if encoder1 and encoder2:
            self._encoder = Decoder(pi, encoder1, encoder2, self.callback)# Encoder(encoder1, encoder2)
            self._ppr = encoderppr
        else:
            raise Exception('Encoder pins set incorrect.')
        # Initializing attributes
        self._value = 0  # Motor output value
        self._angle0 = 0  # Initial angular position
        self.pos = 0

    def callback(self, way):
      self.pos += way
    #   print("pos={}".format(self.pos))

    def __del__(self):
        """
        Class destructor.
        
        """
        # Releasing GPIO pins
        self._motor.close()
        if self._encoder:
            time.sleep(0.1)  # Added this to avoid segmentation fault :(
            self._encoder.cancel()

    @property
    def value(self):
        """
        Contains the motor output level (`read only`).
        Values can be between ``-1`` (full speed backward) and ``1`` (full
        speed forward), with ``0`` being stopped.

        """
        return self._value

    @value.setter
    def value(self, _):
        print('"value" is a read only attribute.')

    def get_angle(self):
        """
        Get the value of the encoder output angle.

        >>> mymotor.get_angle()

        """
        
        if self._encoder:
            # angle = 360 / self._ppr * self._encoder.getValue() - self._angle0
            angle = 360 / self._ppr * self.pos - self._angle0
        else:
            angle = None
        return angle

    def reset_angle(self):
        """
        Reset the encoder output angle.

        >>> mymotor.reset_angle()

        """
        
        if self._encoder:
            #self._angle0 = 360 / self._ppr * self._encoder.getValue()
            self._angle0 = 360 / self._ppr * self.pos

    def set_output(self, output):
        """
        Set motor output.

        :param output: The PWM duty cycle value between ``-1`` and ``1``.
            A value of ``0`` stops the motor.
        :type output: float

        :param brake: The motor brake option used when duty cycle is zero.
            Brake is applied when ``True``. Motor is floating when ``False``.
        :type brake: bool

        Set output to ``0.5``:

            >>> mymotor.set_output(0.5)

        Set output to ``0.25`` (reverse rotation):

            >>> mymotor.set_output(-0.25)

        Stop motor and apply brake:

            >>> mymotor.set_output(0, brake=True)

        """
        # Limiting output
        if output > 1:
            output = 1
        elif output < -1:
            output = -1
        # Forward rotation
        if output > 0:
                self._motor.forward(output)
                
        # Backward rotation
        elif output < 0:
                self._motor.backward(-output)

        # Stop motor
        elif output == 0:
            self._motor.stop()
        # Updating output value property
        self._value = output


class PID:
    """
    The class to represent a discrete PID controller.

    For more information about this class go to:
    https://thingsdaq.org/2022/04/07/digital-pid-controller/


    Create a PID controller:

        >>> Ts = 0.01
        >>> kp = 0.15
        >>> ki = 0.35
        >>> kd = 0.01
        >>> mypid = PID(Ts, kp, ki, kd)

    :param Ts: The sampling period of the execution loop.
    :type Ts: float

    :param kp: The PID proportional gain.
    :type kp: float

    :param ki: The PID integral gain.
    :type ki: float

    :param kd: The PID derivative gain.
    :type kd: float

    :param umax: The upper bound of the controller output saturation.
        Defalt value is ``1``.
    :type umax: float

    :param umin: The lower bound of the controller output saturation.
        Defalt value is ``-1``.
    :type umin: float

    :param tau: The derivative term low-pass filter response time (s).
        Defalt value is ``0``.
    :type tau: float

    """
    def __init__(self, Ts, kp, ki, kd, umax=1, umin=-1, tau=0):
        """
        Class constructor.

        """
        self._Ts = Ts  # Sampling period (s)
        self._kp = kp  # Proportional gain
        self._ki = ki  # Integral gain
        self._kd = kd  # Derivative gain
        self._umax = umax  # Upper output saturation limit
        self._umin = umin  # Lower output saturation limit
        self._tau = tau  # Derivative term filter time constant (s)
        #
        self._eprev = [0, 0]  # Previous errors e[n-1], e[n-2]
        self._uprev = 0  # Previous controller output u[n-1]
        self._udfiltprev = 0  # Previous filtered value

    def control(self, xsp, x, uff=0):
        """
        Calculate PID controller output.

        :param xsp: The set point value at the time step.
        :type xsp: float

        :param x: The actual value at the time step.
        :type x: float

        :param uff: The feed-forward value at the time step.
            Default value is ``0``.
        :type uff: float

        """
        # Calculating error
        e = xsp - x
        # Calculating proportional term
        up = self._kp * (e - self._eprev[0])
        # Calculating integral term (with anti-windup)
        ui = self._ki*self._Ts * e
        if (self._uprev+uff >= self._umax) or (self._uprev+uff <= self._umin):
            ui = 0
        # Calculating derivative term
        ud = self._kd/self._Ts * (e - 2*self._eprev[0] + self._eprev[1])
        # Filtering derivative term
        udfilt = (
            self._tau/(self._tau+self._Ts)*self._udfiltprev +
            self._Ts/(self._tau+self._Ts)*ud
        )
        # Calculating PID controller output
        u = self._uprev + up + ui + udfilt + uff
        # Updating previous time step errors
        self._eprev[1] = self._eprev[0]
        self._eprev[0] = e
        # Updating previous time step output value
        self._uprev = u - uff
        # Updating previous time step derivative term filtered value
        self._udfiltprev = udfilt
        # Limiting output (just to be safe)
        if u < self._umin:
            u = self._umin
        elif u > self._umax:
            u = self._umax
        # Returning controller output at current time step
        return u