from ultralytics import YOLO
 
if __name__ == '__main__':
    model = YOLO('yolo11n.yaml')
    results = model.train(
        data="D:/Racing/D_Robot/dev_ws/src/model_train/coco.yaml",
        epochs=100,
        imgsz=640,
        workers=8,
        batch=96,
        device='cuda:0',
        name='yolo11_train_img_640',
    )
