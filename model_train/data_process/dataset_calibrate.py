import os
from glob import glob
from ultralytics import YOLO
import cv2

model = YOLO('D:/Racing/D_Robot/model_train/data_process/data/best.pt')
img_list = glob('E:/Dataset/obstacle/obstacle_unlabeled/*.jpg')

label_dir = 'labels'
os.makedirs(label_dir, exist_ok=True) 

for img_path in img_list:
    img = cv2.imread(img_path)
    h, w = img.shape[:2]
    results = model(img)
    yolo_txt_lines = []
    for box in results[0].boxes:
        x1, y1, x2, y2 = box.xyxy[0]
        cls = int(box.cls[0])
        x_center = ((x1 + x2) / 2) / w
        y_center = ((y1 + y2) / 2) / h
        width = (x2 - x1) / w
        height = (y2 - y1) / h
        line = f"{cls} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}"
        yolo_txt_lines.append(line)

    base_name = os.path.splitext(os.path.basename(img_path))[0]
    txt_path = os.path.join(label_dir, base_name + ".txt")
    with open(txt_path, "w") as f:
        f.write('\n'.join(yolo_txt_lines))
    print(f"Saved {txt_path}")

print("All images processed and YOLO format labels saved.")
