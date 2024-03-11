import RPi.GPIO as GPIO
from time import sleep

# Motor controller pin
# Motor situation
STOP  = 0
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
# Echo sensor pin
trigger = 23
echo = 24
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
def setMotorContorl(pwm, INA, INB, speed, stat):
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
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        setMotorContorl(pwmB, IN3, IN4, speed, stat)
# double Motor control function
def setIntegratedContorl(stat):
    if stat == FORWARD:
        setMotor(CH1,100,FORWARD)
        setMotor(CH2,100,FORWARD)
    elif stat == BACKWORD:
        setMotor(CH1,100,BACKWORD)
        setMotor(CH2,100,BACKWORD)
    elif stat == STOP:
        setMotor(CH1,100,STOP)
        setMotor(CH2,100,STOP)
    elif stat == RIGHT:
        setMotor(CH1,100,FORWARD)
        setMotor(CH2,100,STOP)
    elif stat == LEFT:
        setMotor(CH1,100,STOP)
        setMotor(CH2,100,FORWARD)
  

# GPIO 모드 설정 
GPIO.setmode(GPIO.BCM)
      
#모터 핀 설정
#핀 설정후 PWM 핸들 얻어옴 
pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)

    
#제어 시작

setIntegratedContorl(FORWARD)
sleep(5)
setIntegratedContorl(BACKWORD)
sleep(5)
setIntegratedContorl(RIGHT)
sleep(5)
setIntegratedContorl(LEFT)
sleep(5)
setIntegratedContorl(STOP)
sleep(5)

# 종료
GPIO.cleanup()