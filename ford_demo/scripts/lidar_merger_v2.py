#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from message_filters import Subscriber, ApproximateTimeSynchronizer

TARGET_FRAME = 'body'  # pick 'body' or another robot-centric frame, NOT 'map'

class LidarMerger(Node):
    def __init__(self):
        super().__init__('lidar_merger')
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        topics = [
            '/lidar_red_pointcloud',
            '/lidar_yellow_pointcloud',
            '/lidar_blue_pointcloud',
            '/lidar_green_pointcloud'
        ]
        subs = [Subscriber(self, PointCloud2, t, qos_profile=10) for t in topics]
        self.sync = ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.05)
        self.sync.registerCallback(self.cb)

        self.pub = self.create_publisher(PointCloud2, '/lidar_combined_pointcloud', 10)

        self._fields = None  # to assert compatibility

    def cb(self, *pcs):
        # 1) Transform each cloud to TARGET_FRAME
        transformed = []
        stamp = pcs[0].header.stamp
        for pc in pcs:
            try:
                tf = self.tf_buffer.lookup_transform(
                    TARGET_FRAME, pc.header.frame_id, rclpy.time.Time.from_msg(stamp),
                    timeout=rclpy.duration.Duration(seconds=0.2)
                )
                transformed.append(do_transform_cloud(pc, tf))
            except Exception as e:
                self.get_logger().warn(f"TF failed: {e}")
                return

        # 2) Verify identical fields/point_step
        f0 = transformed[0]
        if self._fields is None:
            self._fields = (tuple((f.name, f.offset, f.datatype, f.count) for f in f0.fields),
                            f0.point_step, f0.is_bigendian)
        else:
            fields, point_step, is_bigendian = self._fields
            for pc in transformed:
                if (tuple((f.name, f.offset, f.datatype, f.count) for f in pc.fields) != fields or
                        pc.point_step != point_step or pc.is_bigendian != is_bigendian):
                    self.get_logger().error("PointCloud2 fields mismatch between sensors!")
                    return

        # 3) Concatenate raw data
        total_points = sum(pc.width * pc.height for pc in transformed)
        data = bytearray()
        for pc in transformed:
            data.extend(pc.data)

        out = PointCloud2()
        out.header.frame_id = TARGET_FRAME
        out.header.stamp = stamp
        out.height = 1
        out.width = total_points
        out.fields = transformed[0].fields
        out.is_bigendian = transformed[0].is_bigendian
        out.point_step = transformed[0].point_step
        out.row_step = out.point_step * out.width
        out.is_dense = all(pc.is_dense for pc in transformed)
        out.data = bytes(data)

        self.pub.publish(out)

def main():
    rclpy.init()
    node = LidarMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
