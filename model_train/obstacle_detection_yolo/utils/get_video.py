import cv2
import time

camera_id = 0 # v4l2-ctl --list-devices

output_filename = 'output.avi'
fps = 30  
width = 640
height = 480

cap = cv2.VideoCapture(camera_id)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
cap.set(cv2.CAP_PROP_FPS, fps)

if not cap.isOpened():
    print("无法打开摄像头")
    exit(-1)

# 定义 VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(output_filename, fourcc, fps, (width, height))

start_time = time.time()
print("开始录像...")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("无法获取帧，录像终止")
        break

    out.write(frame)


    if time.time() - start_time > 120:
        print("录像结束")
        break

cap.release()
out.release()
cv2.destroyAllWindows()