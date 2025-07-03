from ultralytics import YOLO

YOLO('/home/jasperxzy/racing/D_Robot/model_train/transfer/best.pt').export(imgsz=640, format='onnx', simplify=False, opset=11)
