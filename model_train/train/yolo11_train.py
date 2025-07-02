from ultralytics import YOLO

model = YOLO("yolo11n.yaml") 

results = model.train(data="dataset.yaml", 
                      epochs=50, 
                      imgsz=640, 
                      batch=0.85, 
                      cache=True, 
                      device="0", 
                      workers=8, 
                      project="runs/train/yolo11n", 
                      name="yolo11n_640_v1")
