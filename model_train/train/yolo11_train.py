from ultralytics import YOLO
 
if __name__ == '__main__':
    model = YOLO('yolov5n.yaml')
    results = model.train(
        data="D:/Racing/D_Robot/model_train/train/coco.yaml",
        epochs=50,
        imgsz=640,
        workers=8,
        batch=96,
        device='cuda:0',
        name='yolov5n_train_img_640_v1',
    )
