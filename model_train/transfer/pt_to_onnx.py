from ultralytics import YOLO

model = YOLO('D:/Racing/D_Robot/runs/detect/yolo11_train_img_640_v2/weights/yolo11_img_640.pt')
model.export(format="onnx", dynamic=True)