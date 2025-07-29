import cv2
import time

def video_capture(camera_id, output_filename, fps=30, width=640, height=480):
    """
    Capture video from a camera and save it to a file.
    从摄像头捕获视频并保存到文件

    :param camera_id: ID of the camera to use (default is 0). 摄像头ID（默认0）
    :param output_filename: Name of the output video file. 输出视频文件名
    :param fps: Frames per second for the video. 每秒帧数
    :param width: Width of the video frames. 帧宽
    :param height: Height of the video frames. 帧高
    """
    cap = cv2.VideoCapture(camera_id)  # Open the camera 打开摄像头
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)  # Set frame width 设置帧宽
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height) # Set frame height 设置帧高
    cap.set(cv2.CAP_PROP_FPS, fps)   # Set frames per second 设置帧率

    if not cap.isOpened():
        print("无法打开摄像头")  # Cannot open camera
        exit(-1)

    fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Define the codec 定义视频编码格式
    out = cv2.VideoWriter(output_filename, fourcc, fps, (width, height))  # Open video writer 打开视频写入器

    start_time = time.time()
    print("开始录像...")  # Start recording

    while cap.isOpened():
        ret, frame = cap.read()  # Read a frame 读取一帧
        if not ret:
            print("无法获取帧，录像终止")  # Can't get frame, stop recording
            break

        out.write(frame)  # Write the frame 写入帧

        # Record for 120 seconds (2 minutes) 录像120秒
        if time.time() - start_time > 120:
            print("录像结束") # Recording finished
            break

    cap.release()  # Release the camera 释放摄像头
    out.release()  # Release the video writer 释放写入器
    cv2.destroyAllWindows()  # Close all OpenCV windows 关闭所有OpenCV窗口

if __name__ == "__main__":
    camera_id = 0 # v4l2-ctl --list-devices (can be used to list devices) 可使用此命令列出设备
    output_filename = '../dataset/raw/output.avi'  # Video file path 视频文件路径
    fps = 30  # Frame rate 帧率
    width = 640  # Frame width 帧宽
    height = 480 # Frame height 帧高

    video_capture(camera_id, output_filename, fps, width, height)