#Python - PWM control (GPIO output, #the motor name)

#M1 --> GPIO13 
#M2 --> GPIO12

import RPi.GPIO as GPIO
# pin numbering 
GPIO.setmode(GPIO.BCM)
mode = GPIO.getmode()
#print(mode) # successfully set BCM 

GPIO.setwarnings(False) #disable warnings 

# set up output channels (required for PWM)
chan_list = [12,13]

# setting the output state 
GPIO.setup(chan_list, GPIO.OUT, initial=GPIO.LOW)

#create a PWM instance: 
M1 = GPIO.PWM(13, 50)
M2 = GPIO.PWM(12, 50)

# # to start 
# M1.start(1)   # where dc is the duty cycle (0.0 <= dc <= 100.0)
# input('Press return to stop:')   # use raw_input for Python 2
# M1.stop()
# GPIO.cleanup()

# # to start 
# M2.start(1)   # where dc is the duty cycle (0.0 <= dc <= 100.0)
# input('Press return to stop:')   # use raw_input for Python 2
# M2.stop()
# GPIO.cleanup()
