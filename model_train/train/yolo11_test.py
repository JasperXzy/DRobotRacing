from ultralytics import YOLO

model = YOLO("D:/Racing/D_Robot/runs/detect/yolo11_train_img_640/weights/best.pt")

results = model("E:/Dataset/obstacle_unlabeled/4509.jpg")

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    result.show()  # display to screen
