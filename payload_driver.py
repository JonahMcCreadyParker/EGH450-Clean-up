#Python - PWM control (GPIO output, #the motor name)

#M1 --> GPIO13 
#M2 --> GPIO12

import RPi.GPIO as GPIO # importing the file
import time 

# pin numbering 
GPIO.setmode(GPIO.BCM)
mode = GPIO.getmode() # To detect which pin numbering system has been set (for example, by another Python module):

# print(mode) # successfully set BCM 

GPIO.setwarnings(False) #disable warnings 

# set up output channels (required for PWM)
chan_list = [12,13]

# # tests the channels to make sure they are connected/reading 
# if GPIO.input(12):
#     print('Input 12 was HIGH') 
# else:
#     print('Input 12 was LOW') # bad not connecting 

# if GPIO.input(13):
#     print('Input 13 was HIGH')
# else:
#     print('Input 13 was LOW')

# setting the output state 
GPIO.setup(chan_list, GPIO.OUT, initial=GPIO.LOW)

#create a PWM instance: 
M1 = GPIO.PWM(13, 50) # 50Hz 
M2 = GPIO.PWM(12, 50)

# at 50hz the period is 1/50 = 20ms 
# duty cycle is pulse width / period * 100
# from measured 0.5ms = 90 degrees, 2.5ms = 180 degrees, therefore 
# 2.5/20 * 100 = 12.5 % 

# to start - we want it to start at 0 
M1.start(2.5) # safe neutral-ish starting point
time.sleep(0.5)
M1.ChangeDutyCycle(12.5)
time.sleep(1.5) # gives it time to action the above command 
M1.stop()

# to start - we want it to start at 0 
M2.start(2.5) # safe neutral-ish starting point
time.sleep(0.5)
M2.ChangeDutyCycle(12.5)
time.sleep(1.5) # gives it time to action the above command 
M2.stop()
GPIO.cleanup()
