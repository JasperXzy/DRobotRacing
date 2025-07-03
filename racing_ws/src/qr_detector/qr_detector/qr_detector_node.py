import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import cv2
import time
import sys

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        # 订阅 /sign4return 话题
        self.sign_sub = self.create_subscription(
            Int32,
            '/sign4return',
            self.sign_callback,
            10)
        
        # 创建图像订阅者（但不会立即激活）
        self.image_sub = None
        
        # 创建二维码内容发布者
        self.publisher_ = self.create_publisher(String, 'qr_code', 10)
        
        # 图像转换工具
        self.bridge = CvBridge()
        
        # QR检测器
        self.qr_detector = cv2.QRCodeDetector()
        
        # 状态标志
        self.activated = False  # 是否激活二维码检测
        self.should_stop = False  # 是否停止节点
        self.last_sign_time = 0  # 上次接收到信号的时间
        
        # 优化参数
        self.process_interval = 0.5  # 处理间隔(秒)
        self.scale_factor = 0.5  # 图像缩放比例
        self.last_process_time = 0
        
        # 信号超时设置
        self.sign_timeout = 5.0  # 5秒内未收到信号则自动停用
        
        self.get_logger().info('QR检测器节点已启动 (等待/sign4return信号)')

    def sign_callback(self, msg):
        """处理/sign4return信号的回调函数"""
        current_time = time.time()
        self.last_sign_time = current_time
        
        if msg.data == 5 and not self.activated:
            # 收到激活信号且当前未激活
            self.get_logger().info('收到激活信号(5)，启动二维码检测')
            self.activated = True
            
            # 创建图像订阅者（如果尚未创建）
            if self.image_sub is None:
                self.image_sub = self.create_subscription(
                    Image,
                    'image_raw',
                    self.image_callback,
                    10)
        
        elif msg.data != 5 and self.activated:
            # 收到非激活信号但当前已激活
            self.get_logger().info('收到非激活信号，停止二维码检测')
            self.activated = False

    def image_callback(self, msg):
        """处理图像的回调函数（仅在激活状态下工作）"""
        if not self.activated or self.should_stop:
            return
            
        current_time = time.time()
        
        # 检查信号超时
        if current_time - self.last_sign_time > self.sign_timeout:
            self.get_logger().warn(f'超过 {self.sign_timeout} 秒未收到信号，自动停用二维码检测')
            self.activated = False
            return
        
        # 按固定频率处理图像
        if current_time - self.last_process_time < self.process_interval:
            return
            
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 缩小图像尺寸
            if self.scale_factor < 1.0:
                h, w = cv_image.shape[:2]
                cv_image = cv2.resize(
                    cv_image, 
                    (int(w * self.scale_factor), int(h * self.scale_factor)),
                    interpolation=cv2.INTER_AREA
                )
            
            # 检测二维码
            retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(cv_image)
            
            # 处理检测结果
            if retval and len(decoded_info) > 0:
                for data in decoded_info:
                    if data:  # 确保数据非空
                        # 发布结果
                        qr_msg = String()
                        qr_msg.data = data
                        self.publisher_.publish(qr_msg)
                        
                        self.get_logger().info(f'检测到二维码: {data}')
                        
                        # 设置停止标志
                        self.should_stop = True
                        
                        # 触发节点停止
                        self.get_logger().info('二维码已识别，节点即将停止')
                        
                        # 创建新线程安全停止节点
                        self.create_timer(0.1, self.stop_node)  # 0.1秒后停止
                        break  # 只处理第一个有效二维码
        
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}', throttle_duration_sec=30)
        finally:
            self.last_process_time = current_time
    
    def stop_node(self):
        """安全停止节点"""
        self.get_logger().info('节点停止中...')
        self.destroy_node()
        rclpy.try_shutdown()
        sys.exit(0)  # 确保进程退出

def main(args=None):
    rclpy.init(args=args)
    qr_detector_node = QRDetectorNode()
    try:
        rclpy.spin(qr_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            qr_detector_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()