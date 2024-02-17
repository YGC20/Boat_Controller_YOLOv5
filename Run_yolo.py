import torch
import cv2

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

while True:
    # Frame read
    ret, frame = cap.read()
    if not ret:
        print("Read frame error")
        break

    # Detect Object use YOLOv5
    results = model(frame)
    # Target Check
    target_results = [result for result in results.pred[0] if result[-1] in target_obj]
    # Visualize results
    frame_with_results = results.render(target_results)[0]
    # Results Visualization
    cv2.imshow('YOLOv5 Object Detection',frame_with_results)

    if not target_results:
        print("No target object")
    else:
        # Convert XY_Position to integers
        for result in target_results:
            result[0:4].int()
        print(target_results)
        x_min = target_results[0][0]
        x_max = target_results[0][2]
        if (x_min>200 and x_max<400):
            print("Object in position")
    
    # Press 'q' Out
    if cv2.waitKey(1)&0xFF == ord('q'):
        break

# Out Webcam
cap.release()
cv2.destroyAllWindows()