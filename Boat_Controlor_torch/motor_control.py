import RPi.GPIO as GPIO
import time

# Motor controller pin
# Motor situation
STOP = 0
FORWARD  = 1
BACKWORD = 2
RIGHT = 3
LEFT = 4
# Motor channel
CH1 = 0
CH2 = 1
# PIN set
# PWM PIN
ENA = 26  #37 pin
ENB = 0   #27 pin
# GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin
# Set Pin(Motor)
def setPinConfig(EN, INA, INB):        
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    # Work PWM 100khz
    pwm = GPIO.PWM(EN, 100) 
    # Stop PWM   
    pwm.start(0) 
    return pwm
# Motor control function
def setMotorControl(pwm, INA, INB, speed, stat):
    # Motor speed control PWM
    pwm.ChangeDutyCycle(speed)  
    if stat == FORWARD:
        GPIO.output(INA, GPIO.HIGH)
        GPIO.output(INB, GPIO.LOW)
    elif stat == BACKWORD:
        GPIO.output(INA, GPIO.LOW)
        GPIO.output(INB, GPIO.HIGH)
    elif stat == STOP:
        GPIO.output(INA, GPIO.LOW)
        GPIO.output(INB, GPIO.LOW)
# Motor control function rapping
def setMotor(ch, speed, stat):
    if ch == CH1:
        setMotorControl(pwmA, IN1, IN2, speed, stat)
    else:
        setMotorControl(pwmB, IN3, IN4, speed, stat)
# double Motor control function
def setIntegratedControl(stat):
    if stat == FORWARD:
        setMotor(CH1,100,FORWARD)
        setMotor(CH2,100,FORWARD)
    elif stat == BACKWORD:
        setMotor(CH1,100,BACKWORD)
        setMotor(CH2,100,BACKWORD)
    elif stat == RIGHT:
        setMotor(CH1,100,FORWARD)
        setMotor(CH2,100,STOP)
    elif stat == LEFT:
        setMotor(CH1,100,STOP)
        setMotor(CH2,100,FORWARD)

# Echo sensor pin
trigger = 23
echo = 24
def get_distance():
    GPIO.output(trigger,True)
    time.sleep(0.00001)
    GPIO.output(trigger,False)
    start_time = time.time()
    stop_time = time.time()
    while GPIO.input(echo) == 0:
        start_time = time.time()
    while GPIO.input(echo) == 1:
        stop_time = time.time()
    elapsed_time = stop_time - start_time
    distance = (elapsed_time * 34300) / 2
    return distance

# Set Control
GPIO.setmode(GPIO.BCM)
GPIO.setup(trigger,GPIO.OUT)
GPIO.setup(echo,GPIO.IN)
# Motor PWM set
pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)
