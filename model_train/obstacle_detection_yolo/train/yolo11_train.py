from ultralytics import YOLO

model = YOLO("yolov8n.yaml") 

results = model.train(data="/home/jasperxzy/racing/D_Robot/model_train/obstacle_detection_yolo/train/dataset.yaml", 
                      epochs=64, 
                      imgsz=640, 
                      batch=0.85, 
                      cache=True, 
                      device="0", 
                      workers=8, 
                      project="model_train/obstacle_detection_yolo/runs/train/yolov8n", 
                      name="yolov8n_640_v1")
