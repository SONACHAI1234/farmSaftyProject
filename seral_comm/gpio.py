import RPi.GPIO as GPIO
import time

channel = 26

GPIO.setmode(GPIO.BCM)
GPIO.setup(channel, GPIO.OUT)


def motor_on(pin):
    GPIO.output(pin, GPIO.HIGH)    # Turn on
    print("Now HIGH")


def motor_off(pin):
    GPIO.output(pin, GPIO.LOW) # Turn off
    print("Now low")

if __name__ == '__main__':
    try:
        motor_on(channel)
        time.sleep(1)
        motor_off(channel)
        time.sleep(1)
#        GPIO.cleanup()
    except KeyBoardInterrupt:
        print ("Hello this is not cleanup")
