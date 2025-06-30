from ultralytics import YOLO

model = YOLO('/home/jasperxzy/dataset/yolo11n.pt')
model.export(format="onnx", dynamic=True)