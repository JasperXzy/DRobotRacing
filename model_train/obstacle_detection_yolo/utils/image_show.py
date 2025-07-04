import cv2

def draw_yolo_boxes(image_path, yolo_label_path, output_path=None):
    image = cv2.imread(image_path)
    height, width = image.shape[:2]
    
    with open(yolo_label_path, 'r') as f:
        lines = f.readlines()
        
    for line in lines:
        parts = line.strip().split()
        if len(parts) != 5:
            continue  

        class_id, x_center, y_center, w, h = map(float, parts)
        x_center = x_center * width
        y_center = y_center * height
        w = w * width
        h = h * height
        
        x1 = int(x_center - w / 2)
        y1 = int(y_center - h / 2)
        x2 = int(x_center + w / 2)
        y2 = int(y_center + h / 2)
        
        color = (0, 255, 0)  
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        label = str(int(class_id))

        cv2.putText(image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    
    if output_path:
        cv2.imwrite(output_path, image)
        print(f'Saved to {output_path}')
    else:
        cv2.imshow("Image with Boxes", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

draw_yolo_boxes("model_train\\data_process\\data\\0003.jpg", "model_train\\data_process\\data\\0003.txt")
