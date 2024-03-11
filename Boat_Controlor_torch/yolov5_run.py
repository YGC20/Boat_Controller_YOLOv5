# YOLOv5 module import
import torch
import cv2
import motor_control as mc

# YOLOv5 model load
model = torch.hub.load('','custom','best.pt',source='local')

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
target_obj = [0,1,2]

while True:
    # Get distance from echo sensor
    distance = mc.get_distance()
    # If distance under 10 turn RIGHT
    if distance < 10:
        mc.setIntegratedControl(3)
    else:
        # Frame read
        ret, frame = cap.read()
        if ret:
            # Detect Object use YOLOv5
            results = model(frame, size=300)
            # Target Check
            target_results = [result for result in results.pred[0] if result[-1] in target_obj]
            # Visualize results
            frame_with_results = results.render(target_results)[0]
            # Results Visualization
            cv2.imshow('YOLOv5 Object Detection',frame_with_results)
            if not target_results:
                print("No target object")
                mc.setIntegratedControl(3)
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
                    mc.setIntegratedControl(1)
                elif (x_max<213):
                    print("Object is left")
                    mc.setIntegratedControl(4)
                elif (x_min>426):
                    print("Object is right")
                    mc.setIntegratedControl(3)
    # Press 'q' Out
    if cv2.waitKey(1)&0xFF == ord('q'):
        break
        
# Out Webcam
cap.release()
cv2.destroyAllWindows()