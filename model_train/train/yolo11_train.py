from ultralytics import YOLO
 
if __name__ == '__main__':
    model = YOLO('yolo11n.yaml')
    results = model.train(
        data="D:/Racing/D_Robot/model_train/train/coco.yaml",
        epochs=100,
        imgsz=640,
        workers=8,
        batch=-1,
        device='cuda:0',
        name='yolo11_train_img_640_v2',
    )
