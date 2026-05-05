#keyboard input
from gpiozero import AngularServo
from time import sleep

# Two servos
M1 = AngularServo(12, min_pulse_width=0.0005, max_pulse_width=0.0025)
M2 = AngularServo(13, min_pulse_width=0.0005, max_pulse_width=0.0025)

negative_angle = 70
positive_angle = -40

# Neutral position
M1.angle = negative_angle
M2.angle = negative_angle
sleep(1)

def deploy_M1():
    print("Deploying M1...")
    M1.angle = positive_angle
    sleep(1)

def deploy_M2():
    print("Deploying M2...")
    M2.angle = positive_angle
    sleep(1)

def neutral():
    print("Returning to neutral...")
    M1.angle = negative_angle
    M2.angle = negative_angle
    sleep(1)

try:
    print("Servo controller ready.")
    print("Press '1' + Enter to deploy M1")
    print("Press '2' + Enter to deploy M2")
    print("Press 'n' + Enter to return to neutral")
    print("Press 'q' + Enter to quit")

    while True:
        user_input = input("> ").strip().lower()

        if user_input == '1':
            deploy_M1()
        elif user_input == '2':
            deploy_M2()
        elif user_input == 'n':
            neutral()
        elif user_input == 'q':
            print("Quitting...")
            break
        else:
            print("Unknown input. Use '1', '2', 'n', or 'q'.")

finally:
    print("Stopping servo signals")
    M1.detach()
    M2.detach()

# from gpiozero import AngularServo
# from time import sleep

# # Create two servos
# M1 = AngularServo(13, min_pulse_width=0.0005, max_pulse_width=0.0025)
# M2 = AngularServo(12, min_pulse_width=0.0005, max_pulse_width=0.0025)

# try:
#     print("Both to -90°")
#     M1.angle = -90
#     M2.angle = -90
#     sleep(1)

#     print("Both to +90°")
#     M1.angle = 90
#     M2.angle = 90
#     sleep(1)

# finally:
#     print("Stopping servo signal")
#     M1.detach()
#     M2.detach()