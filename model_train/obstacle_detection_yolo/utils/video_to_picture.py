import cv2

def video_to_picture(video_path, output_folder, frame_interval=1):
    """
    Extract frames from a video file and save them as images.

    :param video_path: Path to the input video file.
    :param output_folder: Folder where the extracted images will be saved.
    :param frame_interval: Interval between frames to save (default is 1, meaning every frame).
    """
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        return

    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break  

        # Save the frame if it matches the interval
        if frame_count % frame_interval == 0:
            image_filename = f"{output_folder}\\frame_{frame_count:04d}.jpg"
            cv2.imwrite(image_filename, frame)
            print(f"Saved {image_filename}")

        frame_count += 1

    cap.release()
    print("Video processing complete.")

if __name__ == "__main__":
    video_path = "D:\Racing\D_Robot\dataset\output.avi"
    output_folder = "D:\Racing\D_Robot\dataset\obstacle"
    frame_interval = 1  # Save every frame
    video_to_picture(video_path, output_folder, frame_interval)
