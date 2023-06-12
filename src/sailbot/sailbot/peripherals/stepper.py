"""
handles sending signal to stepper driver to move stepper a set number of steps
"""
import sys
import time
import RPi.GPIO as GPIO

class stepperDriver(object):

    def __init__(self, direction_pin, step_pin):
        """ class init method 3 inputs
        (1) direction type=int , help=GPIO pin connected to DIR pin of IC
        (2) step_pin type=int , help=GPIO pin connected to STEP of IC
        """
        self.direction_pin = direction_pin
        self.step_pin = step_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

    def turn(self, clockwise=False,
                 steps=800, stepdelay=.001, verbose=False, initdelay=.00, rampTo = .001):
        """ motor_go,  moves stepper motor based on 6 inputs
         (1) clockwise, type=bool default=False
         help="Turn stepper counterclockwise"

         (2) steps, type=int, default=200, help=Number of steps sequence's
         to execute. Default is one revolution , 200 in Full mode.
         (3) stepdelay, type=float, default=0.05, help=Time to wait
         (in seconds) between steps.
         (4) verbose, type=bool  type=bool default=False
         help="Write pin actions",
         (5) initdelay, type=float, default=1mS, help= Intial delay after
         GPIO pins initialized but before motor is moved.
        """
        # setup GPIO
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.output(self.direction_pin, clockwise)
        try:

            time.sleep(initdelay)

            for i in range(steps):
                #print(F'loop {i}')
                GPIO.output(self.step_pin, True)
                time.sleep(stepdelay)
                GPIO.output(self.step_pin, False)
                time.sleep(stepdelay)

                if stepdelay > rampTo:
                    stepdelay -= (stepdelay - rampTo) / 500

                if verbose:
                    print("Steps count {}".format(i))

        except KeyboardInterrupt:
            print("User Keyboard Interrupt : RpiMotorLib:")
        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("RpiMotorLib  : Unexpected error:")
        else:
            # print report status
            if verbose:
                print("\nRpiMotorLib, Motor Run finished, Details:.\n")
                #print("Motor type = {}".format(self.motor_type))
                print("Clockwise = {}".format(clockwise))
                #print("Step Type = {}".format(steptype))
                print("Number of steps = {}".format(steps))
                print("Step Delay = {}".format(stepdelay))
                print("Intial delay = {}".format(initdelay))
                #print("Size of turn in degrees = {}".format(degree_calc(steps, steptype)))
        finally:
            # cleanup
            GPIO.output(self.step_pin, False)
            GPIO.output(self.direction_pin, False)

                
if __name__ == '__main__':
    dirPin = 17
    stepPin = 27
    
    highPin = 20
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    
    drv = stepperDriver(dirPin, stepPin)
    
    drv.turn(verbose = True, stepdelay = .001)
    
    #GPIO.output(stepPin, True)
    
