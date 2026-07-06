#!/usr/bin/env python3
import rclpy 
from rclpy.node  import Node 
from sensor_msgs.msg  import Image
from std_msgs.msg  import String, Int32
from cv_bridge import CvBridge 
import cv2 
import time
import sys 
from pyzbar import pyzbar
from origincar_msg.msg  import Sign
from rclpy.qos  import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from ai_msgs.msg  import PerceptionTargets 
from ctypes import cdll, c_void_p, c_int, POINTER, Structure 
import numpy as np 
 
class ZBarScanner:
    """封装 ZBar 的 ctypes 调用，支持密度配置"""
    def __init__(self):
        # 加载 ZBar 库 
        try:
            self.zbar  = cdll.LoadLibrary("libzbar.so")   # Linux/Mac 
        except OSError:
            try:
                self.zbar  = cdll.LoadLibrary("zbar.dll")   # Windows
            except OSError as e:
                raise RuntimeError(f"无法加载 ZBar 库: {e}")
 
        # 定义简化版 ZBar 结构体 
        class zbar_image(Structure):
            _fields_ = [("dummy", c_int)]
 
        self.zbar_image_type  = zbar_image 
        self._init_scanner()
 
    def _init_scanner(self):
        """初始化扫描器并配置密度"""
        self.zbar.zbar_image_scanner_create.restype  = c_void_p 
        self.scanner  = self.zbar.zbar_image_scanner_create() 
 
        # 配置扫描密度 
        CFG_X_DENSITY = 0x100
        CFG_Y_DENSITY = 0x101
        self.zbar.zbar_image_scanner_set_config(self.scanner,  0, CFG_X_DENSITY, c_int(2))
        self.zbar.zbar_image_scanner_set_config(self.scanner,  0, CFG_Y_DENSITY, c_int(2))
 
    def scan(self, cv_image):
        """扫描图像中的二维码（返回解码结果列表）"""
        if cv_image is None:
            return []
 
        # 转换为灰度图
        if len(cv_image.shape)  == 3:
            gray = cv2.cvtColor(cv_image,  cv2.COLOR_BGR2GRAY)
        else:
            gray = cv_image
 
        height, width = gray.shape 
        results = []
 
        try:
            # 创建 ZBar 图像对象 
            self.zbar.zbar_image_create.restype  = POINTER(self.zbar_image_type) 
            zimg = self.zbar.zbar_image_create() 
            self.zbar.zbar_image_set_size(zimg,  width, height)
 
            # 传递图像数据 
            c_array = np.ascontiguousarray(gray,  dtype=np.uint8) 
            self.zbar.zbar_image_set_data(zimg,  c_array.ctypes.data_as(POINTER(c_void_p)),  width * height, None)
 
            # 执行扫描
            self.zbar.zbar_scan_image(self.scanner,  zimg)
 
            # TODO: 实际应用中需解析扫描结果（此处简化）
            # 示例：假设返回第一个解码结果
            # 实际应调用 zbar_image_first_symbol() 等函数遍历结果
 
            results.append("Dummy  QR Content")  # 替换为实际解析逻辑
 
        finally:
            if 'zimg' in locals():
                self.zbar.zbar_image_destroy(zimg) 
 
        return results
 
    def __del__(self):
        if hasattr(self, 'scanner'):
            self.zbar.zbar_image_scanner_destroy(self.scanner) 
 
class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        # 初始化 ZBar 扫描器（带密度控制）
        try:
            self.zbar_scanner  = ZBarScanner()
            self.get_logger().info("ZBar  扫描器初始化成功（已启用密度控制）")
        except Exception as e:
            self.get_logger().error(f"ZBar  初始化失败: {e}, 回退到 pyzbar")
            self.zbar_scanner  = None 
 
        # ... [其余初始化代码保持不变] ...
 
    def image_callback(self, msg):
        """处理图像的回调函数（整合 ZBar 和 pyzbar）"""
        if not self.activated: 
            return 
 
        current_time = time.time() 
        if current_time - self.last_process_time  < self.process_interval: 
            return
 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg,  'bgr8')
            
            # 优先使用 ZBar（如果可用）
            if self.zbar_scanner: 
                qr_contents = self.zbar_scanner.scan(cv_image) 
                for content in qr_contents:
                    self._process_qr_content(content)
            else:
                # 回退到 pyzbar 
                barcodes = pyzbar.decode(cv_image,  symbols=[pyzbar.ZBarSymbol.QRCODE])
                for barcode in barcodes:
                    self._process_qr_content(barcode.data.decode("utf-8")) 
 
        except Exception as e:
            self.get_logger().error(f' 图像处理错误: {e}', throttle_duration_sec=5)
        finally:
            self.last_process_time  = current_time
 
    def _process_qr_content(self, content):
        """统一处理二维码内容"""
        self.get_logger().info(f' 检测到二维码: {content}')
        
        qr_msg = String()
        qr_msg.data  = content
        self.publisher_.publish(qr_msg) 
 
        if content.isdigit(): 
            qr_value = int(content)
            self.Signmsg.sign_data  = 3 if qr_value % 2 == 1 else 4 
            self.sign_switch_pub.publish(self.Signmsg) 
            self.send_stop_command() 
            self.activated  = False 
 
# ... [main() 函数保持不变] ...
    def send_stop_command(self):
        """发送停止命令到机器人"""
        stop_msg = Int32()
        stop_msg.data = 5
        self.sign4return_pub.publish(stop_msg)
        self.get_logger().info('已发布停止命令(5)')


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
