from ultralytics import YOLO
 
if __name__ == '__main__':
    model = YOLO('yolo11n.yaml')
    results = model.train(
        data="/home/jasperxzy/racing/D_Robot/model_train/train/coco.yaml",
        epochs=50,
        imgsz=640,
        workers=8,
        batch=96,
        device='cuda:0',
        name='yolo11_train_img_640_v3',
    )
