import RPi.GPIO as GPIO
import time

motor1 = 17
motor2 = 18

trigger_pin = 23
echo_pin = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(motor1,GPIO.OUT)
GPIO.setup(motor2,GPIO.OUT)
GPIO.setup(trigger_pin,GPIO.OUT)
GPIO.setup(echo_pin,GPIO.IN)

def get_distance():
    GPIO.output(trigger_pin,True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin,False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo_pin) == 0:
        start_time = time.time()
    
    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()
    
    elapsed_time = stop_time - stop_time
    distance = (elapsed_time * 34300) / 2

    return distance

try:
    while True:
        distance = get_distance()

        if distance < 10:
            GPIO.output(motor1_pin1, GPIO.HIGH)
            GPIO.output(motor1_pin2, GPIO.LOW)
            GPIO.output(motor1_pin1, GPIO.LOW)
            GPIO.output(motor1_pin2, GPIO.HIGH)
        else:
            GPIO.output(motor1_pin1, GPIO.HIGH)
            GPIO.output(motor1_pin2, GPIO.LOW)
            GPIO.output(motor1_pin1, GPIO.LOW)
            GPIO.output(motor1_pin2, GPIO.HIGH)
            
except KeyboardInterrupt:
    GPIO.cleanup()