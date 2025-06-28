from ultralytics import YOLO
 
if __name__ == '__main__':
    model = YOLO('yolov8n.yaml')
    results = model.train(
        data="D:/Racing/D_Robot/dev_ws/src/model_train/coco.yaml",
        epochs=100,
        imgsz=640,
        workers=1,
        batch=4,
        device='cuda:0',
    )
    