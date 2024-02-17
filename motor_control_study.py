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
    
    elapsed_time = stop_time - start_time
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


''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''


import RPi.GPIO as GPIO
from time import sleep

# ���� ����
STOP  = 0
FORWARD  = 1
BACKWORD = 2

# ���� ä��
CH1 = 0
CH2 = 1

# PIN ����
HIGH = 1
LOW = 0

# ���� �� ����
#PWM PIN
ENA = 26  #37 pin
ENB = 0   #27 pin

#GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin

# �� ���� �Լ�
def setPinConfig(EN, INA, INB):        
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    
    # 100khz �� PWM ���� ��Ŵ 
    pwm = GPIO.PWM(EN, 100) 
    
    # �켱 PWM ����.   
    pwm.start(0) 
    return pwm

# ���� ���� �Լ�
def setMotorContorl(pwm, INA, INB, speed, stat):

    #���� �ӵ� ���� PWM
    pwm.ChangeDutyCycle(speed)  
    
    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)
        
    #�ڷ�
    elif stat == BACKWORD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)
        
    #����
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)

        
# ���� �����Լ� �����ϰ� ����ϱ� ���� �ѹ��� ����(����)
def setMotor(ch, speed, stat):
    if ch == CH1:
        #pwmA�� �� ���� �� pwm �ڵ��� ���� ���� ���̴�.
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        #pwmB�� �� ���� �� pwm �ڵ��� ���� ���� ���̴�.
        setMotorContorl(pwmB, IN3, IN4, speed, stat)
  

# GPIO ��� ���� 
GPIO.setmode(GPIO.BCM)
      
#���� �� ����
#�� ������ PWM �ڵ� ���� 
pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)

    
#���� ����

# ������ 80���� �ӵ���
setMotor(CH1, 80, FORWARD)
setMotor(CH2, 80, FORWARD)
#5�� ���
sleep(5)

# �ڷ� 40���� �ӵ���
setMotor(CH1, 40, BACKWORD)
setMotor(CH2, 40, BACKWORD)
sleep(5)

# �ڷ� 100���� �ӵ���
setMotor(CH1, 100, BACKWORD)
setMotor(CH2, 100, BACKWORD)
sleep(5)

#���� 
setMotor(CH1, 80, STOP)
setMotor(CH2, 80, STOP)

# ����
GPIO.cleanup()