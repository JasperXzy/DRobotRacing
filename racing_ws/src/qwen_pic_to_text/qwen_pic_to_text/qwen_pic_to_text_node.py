#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import cv2
import base64
import numpy as np
import os
from openai import OpenAI
import threading

class PicToTextNode(Node):
    def __init__(self):
        super().__init__('pic_to_text_node')
        
        self.client = OpenAI(
            base_url="http://192.168.10.138:1234/v1", 
            api_key="sk-no-key-required"         
        )

        # 添加信号订阅器
        self.signal_sub = self.create_subscription(
            Int32,
            '/sign4return',
            self.signal_callback,
            1
        )
        
        # 创建结果发布者
        self.result_publisher = self.create_publisher(
            String,
            '/pic_info',
            1
        )
        
        # 初始化图像订阅器为None
        self.image_sub = None
        self.ready = False
        
        self.bridge = CvBridge()
        self.image_received = False     # 用于确保只处理第一张收到的图片
        self.lock = threading.Lock()    # 线程锁，防止重复处理
        
        self.get_logger().info("图生文节点已启动，等待接收启动信号...")

    def signal_callback(self, msg):
        """处理启动信号的回调函数"""
        # 只有在未准备状态下接收到 -1 信号才创建图像订阅器
        if msg.data == -1 and not self.ready:
            self.ready = True
            self.get_logger().info("接收到启动信号(-1)，开始订阅图像话题")
            
            # 创建图像订阅器
            self.image_sub = self.create_subscription(
                CompressedImage,
                '/image', 
                self.image_callback,
                1
            )

    def image_callback(self, msg):
        # 只有在准备状态下才处理图像
        if not self.ready:
            self.get_logger().debug("收到图像但节点未准备好，忽略。")
            return
            
        with self.lock:
            # 只处理接收到的第一张图片，避免重复触发AI推理
            if self.image_received:
                self.get_logger().debug("已接收过图像并正在处理或已处理，忽略后续图像。")
                return
            
            self.image_received = True # 设置标志，表示已收到并开始处理
            self.get_logger().info("收到图像，开始处理...")
            
            try:
                # 将 CompressedImage 消息数据从字节流解码为 OpenCV 图像
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                _, buffer = cv2.imencode('.jpg', cv_image)
                base64_image = base64.b64encode(buffer).decode('utf-8')
                
                # 调用 LM Studio 中加载的模型进行描述
                description = self.describe_image(base64_image)
                
                # 输出完整回复
                self.get_logger().info("\n[LM Studio Model Response]:\n" + description)
                
                # 发布结果到 /pic_info 话题
                result_msg = String()
                result_msg.data = description
                self.result_publisher.publish(result_msg)
                self.get_logger().info("已发布结果到 /pic_info 话题")
                
            except Exception as e:
                self.get_logger().error(f"图像处理或 API 调用失败: {str(e)}")
            finally:
                # 无论成功或失败，处理完成后都准备关闭节点
                self.get_logger().info("图像处理流程完成，节点将在2秒后关闭。")
                # 设置一个定时器，在指定时间后调用 shutdown_node
                self.timer = self.create_timer(2.0, self.shutdown_node)

    def describe_image(self, base64_image):
        """
        向 LM Studio 的 OpenAI 兼容 API 发送请求以描述图像。
        """
        try:
            completion = self.client.chat.completions.create(
                model="google/gemma-3-4b",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                            },
                            {"type": "text", "text": "描述立牌上图片信息，使用一段话输出，包括人物环境地点细节，其背景是在医疗环境，大约50字"},
                        ],
                    }
                ],
                stream=False
            )

            return completion.choices[0].message.content
            
        except Exception as e:
            return f"API调用失败: {str(e)}. 请检查 LM Studio 是否运行，并已加载正确的（多模态）模型。"

    def shutdown_node(self):
        """关闭节点，并确保清理资源"""
        if hasattr(self, 'timer') and self.timer is not None:
            self.destroy_timer(self.timer)
            self.get_logger().debug("定时器已销毁。")
        
        if hasattr(self, 'image_sub') and self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.get_logger().debug("图像订阅器已销毁。")

        if hasattr(self, 'signal_sub') and self.signal_sub is not None:
            self.destroy_subscription(self.signal_sub)
            self.get_logger().debug("信号订阅器已销毁。")

        self.get_logger().info('节点即将关闭。')
        self.destroy_node()
        rclpy.shutdown() 

def main(args=None):
    rclpy.init(args=args) 
    node = PicToTextNode()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        node.get_logger().info("接收到键盘中断，正在关闭节点。")
    finally:
        if rclpy.ok() and node.executor.is_spinning_up_to_date():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()