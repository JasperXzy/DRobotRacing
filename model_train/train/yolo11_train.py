from ultralytics import YOLO
 
if __name__ == '__main__':
    model = YOLO('yolov5s.yaml')
    results = model.train(
        data="D:/Racing/D_Robot/model_train/train/coco.yaml",
        epochs=50,
        imgsz=640,
        workers=8,
        batch=0.7,
        device='cuda:0',
        name='yolov5s_train_img_640_v1',
    )
