import cv2
from ultralytics import YOLO


model = YOLO('D:/Racing/D_Robot/models/yolo11_img_640.onnx')
class_names = ['obstacle'] 

video_path = 'E:/Dataset/obstacle/obstacle_raw/output.avi'
cap = cv2.VideoCapture(video_path)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
fps = cap.get(cv2.CAP_PROP_FPS)
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

out = cv2.VideoWriter('output.mp4', fourcc, fps, (width, height))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()    # [x1, y1, x2, y2]
        confs = result.boxes.conf.cpu().numpy()    # 置信度
        clss = result.boxes.cls.cpu().numpy()      # 分类id

        for box, conf, cls in zip(boxes, confs, clss):
            x1, y1, x2, y2 = map(int, box)
            label = f"{class_names[int(cls)]}: {conf:.2f}"
            color = (0, 255, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.imshow('YOLO11 Detection', frame)

    out.write(frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
print('Saved output.mp4')
