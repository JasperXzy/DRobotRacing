from ultralytics import YOLO

model = YOLO("D:/Racing/D_Robot/dev_ws/src/model_train/best.pt")

results = model("D:/Racing/D_Robot/dev_ws/src/model_train/0819.jpg")

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    result.show()  # display to screen
    result.save(filename="result.jpg")  # save to disk
    