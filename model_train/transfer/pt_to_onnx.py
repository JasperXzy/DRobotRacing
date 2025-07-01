from ultralytics import YOLO

model = YOLO('D:/Racing/D_Robot/runs/detect/yolov5n_train_img_640_v1/weights/yolov5n.pt')
model.export(format="onnx", opset=11)
