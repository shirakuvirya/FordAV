#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import copy

class LidarMerger(Node):
    def __init__(self):
        super().__init__('lidar_merger')
        tops = [
            '/lidar_red_pointcloud',
            '/lidar_yellow_pointcloud',
            '/lidar_blue_pointcloud',
            '/lidar_green_pointcloud'
        ]
        self.buff = {t: None for t in tops}
        for t in tops:
            self.create_subscription(
                PointCloud2, t,
                lambda msg, to=t: self.cb(to, msg),
                10)
        self.pub = self.create_publisher(PointCloud2, '/lidar_combined_pointcloud', 10)
        self.create_timer(0.1, self.publish_combined)

    def cb(self, topic, msg):
        self.buff[topic] = msg

    def publish_combined(self):
        if any(v is None for v in self.buff.values()):
            return
        base = copy.deepcopy(next(iter(self.buff.values())))
        pts = sum(pc.width for pc in self.buff.values())
        base.width = pts
        base.row_step = base.point_step * pts
        data = bytearray()
        for pc in self.buff.values():
            data.extend(pc.data)
        base.data = bytes(data)
        base.header.stamp = next(iter(self.buff.values())).header.stamp
        base.header.frame_id = 'map'
        self.pub.publish(base)

def main(args=None):
    rclpy.init(args=args)
    node = LidarMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
