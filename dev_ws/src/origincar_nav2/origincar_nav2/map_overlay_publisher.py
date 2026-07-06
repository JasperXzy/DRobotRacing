#!/usr/bin/env python3
"""
map_overlay_publisher.py — 地图图片叠加发布节点

将 PNG 标注图片叠到 RViz2 Map 显示之上，支持彩色可视化。
图片自动缩放对齐到 PGM 地图尺寸（使用 my_map.yaml 的 resolution/origin）。

发布:
  /map_overlay/image   → RViz2 Image 面板
  /map_overlay/marker  → RViz2 3D 视图半透明叠加
  /map_overlay/grid    → nav_msgs/OccupancyGrid（Map 显示叠加）

用法:
  # 自动从 my_map.yaml 读取参数
  ros2 run origincar_nav2 map_overlay_publisher

  # 自定义
  ros2 run origincar_nav2 map_overlay_publisher \
    --ros-args -p image_file:=/path/to/overlay.png \
               -p yaml_file:=/path/to/my_map.yaml \
               -p alpha:=0.5
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA, Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory


def load_yaml_params(yaml_path):
    """从 map YAML 读取 resolution / origin"""
    params = {}
    with open(yaml_path, 'r') as f:
        for line in f:
            line = line.strip()
            if ':' in line:
                k, v = line.split(':', 1)
                k, v = k.strip(), v.strip()
                if k == 'resolution':
                    params['resolution'] = float(v)
                elif k == 'origin':
                    v = v.strip('[]').split(',')
                    params['origin'] = [float(x) for x in v]
                elif k == 'image':
                    params['image'] = v.strip()
                elif k == 'negate':
                    params['negate'] = int(v)
    return params


class MapOverlayPublisher(Node):
    def __init__(self):
        super().__init__('map_overlay_publisher')

        # ---- 参数声明 ----
        self.declare_parameter('image_file', '')
        self.declare_parameter('yaml_file', '')
        self.declare_parameter('resolution', 0.025)
        self.declare_parameter('origin_x', -2.62)      # 与 my_map.yaml 匹配
        self.declare_parameter('origin_y', -2.96)
        self.declare_parameter('pgm_width', 359)        # PGM 像素宽
        self.declare_parameter('pgm_height', 358)       # PGM 像素高
        self.declare_parameter('alpha', 0.5)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('publish_grid', True)

        image_file = self.get_parameter('image_file').value
        yaml_file = self.get_parameter('yaml_file').value
        self.resolution = self.get_parameter('resolution').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.pgm_w = self.get_parameter('pgm_width').value
        self.pgm_h = self.get_parameter('pgm_height').value
        self.alpha = self.get_parameter('alpha').value
        self.publish_image = self.get_parameter('publish_image').value
        self.publish_marker = self.get_parameter('publish_marker').value
        self.publish_grid = self.get_parameter('publish_grid').value

        # ---- 自动从 YAML 读取参数 ----
        pkg_share = get_package_share_directory('origincar_nav2')

        if not yaml_file:
            yaml_file = os.path.join(pkg_share, 'map', 'my_map.yaml')
            if os.path.exists(yaml_file):
                self.get_logger().info(f'自动检测到 YAML: {yaml_file}')
        if not image_file:
            image_file = os.path.join(pkg_share, 'map', 'my_map.png')
            if os.path.exists(image_file):
                self.get_logger().info(f'自动检测到图片: {image_file}')

        if os.path.exists(yaml_file):
            yaml_params = load_yaml_params(yaml_file)
            self.resolution = yaml_params.get('resolution', self.resolution)
            self.origin_x = yaml_params['origin'][0]
            self.origin_y = yaml_params['origin'][1]
            # 从 PGM 文件读取实际尺寸
            pgm_name = yaml_params.get('image', 'my_map.pgm')
            pgm_path = os.path.join(os.path.dirname(yaml_file), pgm_name)
            if os.path.exists(pgm_path):
                with open(pgm_path, 'rb') as pf:
                    assert pf.readline().strip() == b'P5'
                    line = pf.readline()
                    while line.startswith(b'#'):
                        line = pf.readline()
                    self.pgm_w, self.pgm_h = map(int, line.split())
            self.get_logger().info(
                f'从 YAML 读取: resolution={self.resolution}, '
                f'origin=({self.origin_x}, {self.origin_y}), '
                f'PGM={self.pgm_w}x{self.pgm_h}')
        else:
            self.get_logger().warn(f'YAML 不存在: {yaml_file}，使用默认参数')

        # ---- 读取图片 ----
        if not os.path.exists(image_file):
            self.get_logger().error(f'图片文件不存在: {image_file}')
            raise FileNotFoundError(f'Image not found: {image_file}')

        self.cv_image = cv2.imread(image_file, cv2.IMREAD_UNCHANGED)
        if self.cv_image is None:
            raise RuntimeError(f'无法读取图片: {image_file}')

        img_h, img_w = self.cv_image.shape[:2]
        self.get_logger().info(f'原始图片: {img_w}x{img_h}px')

        # ---- 缩放到 PGM 尺寸 ----
        self.overlay_image = cv2.resize(
            self.cv_image, (self.pgm_w, self.pgm_h),
            interpolation=cv2.INTER_LANCZOS4)
        self.get_logger().info(f'缩放后: {self.overlay_image.shape[1]}x{self.overlay_image.shape[0]}px')

        # 计算实际世界尺寸
        self.real_w = self.pgm_w * self.resolution
        self.real_h = self.pgm_h * self.resolution
        self.get_logger().info(
            f'世界尺寸: {self.real_w:.2f}m x {self.real_h:.2f}m '
            f'(origin=({self.origin_x:.2f}, {self.origin_y:.2f}))')

        self.bridge = CvBridge()

        # ---- 发布者 ----
        if self.publish_image:
            self.image_pub = self.create_publisher(RosImage, '/map_overlay/image', 10)
            self.image_timer = self.create_timer(1.0, self._publish_image)

        if self.publish_marker:
            self.marker_pub = self.create_publisher(Marker, '/map_overlay/marker', 10)
            self.marker_timer = self.create_timer(0.5, self._publish_marker)

        if self.publish_grid:
            self.grid_pub = self.create_publisher(OccupancyGrid, '/map_overlay/grid', 10)
            self.grid_timer = self.create_timer(1.0, self._publish_grid)

        self.get_logger().info('MapOverlayPublisher 已启动')
        self.get_logger().info('  RViz2: 添加 Map 显示, Topic=/map_overlay/grid, ColorScheme=costmap')
        self.get_logger().info('  RViz2: 添加 Image 显示, Topic=/map_overlay/image')

    def _publish_image(self):
        """发布 sensor_msgs/Image → RViz2 Image 面板"""
        # 发布缩放后的图片 (与 PGM 对齐)
        # 确保是 3 通道 BGR (去 alpha)
        if len(self.overlay_image.shape) == 3 and self.overlay_image.shape[2] == 4:
            display_img = cv2.cvtColor(self.overlay_image, cv2.COLOR_BGRA2BGR)
        else:
            display_img = self.overlay_image
        ros_img = self.bridge.cv2_to_imgmsg(display_img, encoding='bgr8')
        ros_img.header.frame_id = 'map'
        ros_img.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(ros_img)

    def _publish_grid(self):
        """发布 nav_msgs/OccupancyGrid → RViz2 Map 显示叠加"""
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()

        # 与 base map 使用相同的 metadata
        grid.info.resolution = self.resolution
        grid.info.width = self.pgm_w
        grid.info.height = self.pgm_h
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # 将彩色图片转为占据值: 非透明像素 → 高代价, 透明像素 → 未知
        img = self.overlay_image
        if len(img.shape) == 3 and img.shape[2] >= 3:
            # BGR 图片: 检测"有内容"的像素 (非纯黑 = 有标注)
            gray = cv2.cvtColor(img[:, :, :3], cv2.COLOR_BGR2GRAY)
            mask = gray > 10  # 非黑像素 = 标注内容
        else:
            # 灰度图: 直接阈值
            mask = img > 10

        data = np.full((self.pgm_h, self.pgm_w), -1, dtype=np.int8)  # 默认未知
        data[mask] = 100  # 标注区域 → 高占据 (在 costmap 配色中呈红色)

        grid.data = data.flatten().tolist()
        self.grid_pub.publish(grid)

    def _publish_marker(self):
        """发布 textured Marker → RViz2 3D 视图地面叠加"""
        # 编码图片为 PNG 纹理
        # 确保 overlay_image 是 3 通道 BGR (去 alpha)
        if len(self.overlay_image.shape) == 3 and self.overlay_image.shape[2] == 4:
            display_img = cv2.cvtColor(self.overlay_image, cv2.COLOR_BGRA2BGR)
        else:
            display_img = self.overlay_image

        success, encoded = cv2.imencode('.png', display_img)
        if not success:
            self.get_logger().error('图片编码失败')
            return

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'map_overlay'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.pose.position.z = 0.002  # 略高于地面
        marker.pose.orientation.w = 1.0

        # 嵌入纹理
        marker.texture_resource = 'embedded_texture://'
        marker.texture.format = 'png'
        marker.texture.data = encoded.tobytes()

        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=self.alpha)

        # 构建两个三角形覆盖整个地图区域
        x0, y0 = self.origin_x, self.origin_y
        x1, y1 = x0 + self.real_w, y0 + self.real_h

        marker.points = [
            Point(x=x0, y=y0, z=0.0),
            Point(x=x1, y=y0, z=0.0),
            Point(x=x1, y=y1, z=0.0),
            Point(x=x0, y=y0, z=0.0),
            Point(x=x1, y=y1, z=0.0),
            Point(x=x0, y=y1, z=0.0),
        ]

        # 半透明顶点色
        marker.colors = [ColorRGBA(r=1.0, g=1.0, b=1.0, a=self.alpha)] * 6
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MapOverlayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()