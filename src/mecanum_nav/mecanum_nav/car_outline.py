"""发布车体线稿轮廓（LineStrip），供 RViz 显示。"""
import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker


class CarOutline(Node):

    def __init__(self):
        super().__init__('car_outline')
        path = os.path.join(get_package_share_directory('mecanum_nav'),
                            'config', 'car_outline.json')
        with open(path, encoding='utf-8') as f:
            self.contours = json.load(f)
        self.declare_parameter('line_width', 0.004)
        self.declare_parameter('z', 0.02)
        self.pub = self.create_publisher(Marker, 'car_outline', 10)
        self.create_timer(1.0, self.publish)
        self.get_logger().info('车体线稿轮廓已加载：%d 条轮廓' % len(self.contours))

    def publish(self):
        width = self.get_parameter('line_width').value
        z = self.get_parameter('z').value
        stamp = self.get_clock().now().to_msg()
        for i, contour in enumerate(self.contours):
            msg = Marker()
            msg.header.frame_id = 'base_link'
            msg.header.stamp = stamp
            msg.ns = 'car_outline'
            msg.id = i
            msg.type = Marker.LINE_STRIP
            msg.action = Marker.ADD
            msg.scale.x = width
            msg.color.r = 1.0
            msg.color.g = 0.55
            msg.color.b = 0.0
            msg.color.a = 1.0
            msg.pose.position.z = z
            msg.points = [Point(x=p[0], y=p[1], z=0.0)
                          for p in contour + [contour[0]]]
            self.pub.publish(msg)


def main():
    rclpy.init()
    node = CarOutline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
