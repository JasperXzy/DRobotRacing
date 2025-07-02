from ultralytics import YOLO

model = YOLO("yolo11n.yaml") 

results = model.train(data="/home/jasperxzy/racing/D_Robot/model_train/train/dataset.yaml", 
                      epochs=50, 
                      imgsz=640, 
                      batch=0.85, 
                      cache=True, 
                      device="0", 
                      workers=8, 
                      project="model_train/train/runs/train/yolo11n", 
                      name="yolo11n_640_v1")
