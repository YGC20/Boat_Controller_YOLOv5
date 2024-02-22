# YOLOv5 module import
import torch
import cv2
# Motor Controller module import
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


# YOLOv5 model load
model = torch.hub.load('','custom','yolov5s.pt',source='local')

# Webcam Open
cap = cv2.VideoCapture(0)
# Webcam Frame set
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
# Webcam FPS set
cap.set(cv2.CAP_PROP_FPS,60)
# Webcam In_Buffersize set
cap.set(cv2.CAP_PROP_BUFFERSIZE,1)

# target_object = bottle(39)
target_obj = [39]

if cap.isOpened():
    while True:
        delay = int(1000/60)
        distance = get_distance()
        
        if distance < 10:
            setIntegratedControl(RIGHT)
        else:
            # Frame read
            ret, frame = cap.read()
            if ret:
                # Detect Object use YOLOv5
                results = model(frame)
                # Target Check
                target_results = [result for result in results.pred[0] if result[-1] in target_obj]
                # Visualize results
                frame_with_results = results.render(target_results)[0]
                # Results Visualization
                cv2.imshow('YOLOv5 Object Detection',frame_with_results)
                cv2.waitKey(delay)

                if not target_results:
                    print("No target object")
                    setIntegratedControl(RIGHT)
                else:
                    # Convert XY_Position to integers
                    for result in target_results:
                        result[0:4].int()
                    # Find best object
                    target_results.sort(key=lambda x:-x[4])
                    print(target_results[0])
                    x_min = target_results[0][0]
                    x_max = target_results[0][2]

                    if (x_min>213 and x_max<426):
                        print("Object in position")
                        setIntegratedControl(FORWARD)
                    elif (x_max<213):
                        print("Object is left")
                        setIntegratedControl(LEFT)
                    elif (x_min>426):
                        print("Object is right")
                        setIntegratedControl(RIGHT)
            else:
                print("Can\'t read frame")
                break
        
        # Press 'q' Out
        if cv2.waitKey(1)&0xFF == ord('q'):
            break
else:
    print("Read frame error")
        
# Out controller Motor
GPIO.cleanup()
# Out Webcam
cap.release()
cv2.destroyAllWindows()




'''
DC Motor control study site
https://uigwonblog.tistory.com/25
'''