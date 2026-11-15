"""Non-blocking ROS 2 PointCloud2 and TF bridge for the MuJoCo thread."""
from queue import Queue, Empty
from threading import Thread
import numpy as np


class LidarRosBridge:
    def __init__(self, model, lidar_id, rate_hz=10.0, publish_map=False):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2, PointField
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

        self.rclpy = rclpy
        self.PointCloud2 = PointCloud2
        self.PointField = PointField
        self.TransformStamped = TransformStamped
        self.owns_context = not rclpy.ok()
        if self.owns_context:
            rclpy.init(args=None)
        self.node = rclpy.create_node('mujoco_lidar_bridge')
        self.points_pub = self.node.create_publisher(
            PointCloud2, '/drone/lidar/points', qos_profile_sensor_data
        )
        self.map_pub = self.node.create_publisher(
            PointCloud2, '/drone/lidar/map_points', qos_profile_sensor_data
        ) if publish_map else None
        self.tf_pub = TransformBroadcaster(self.node)
        self.static_tf_pub = StaticTransformBroadcaster(self.node)
        self.latest = Queue(maxsize=1)
        self.publish_map = publish_map
        self.fields = [
            PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
            for name, offset in (('x', 0), ('y', 4), ('z', 8))
        ]
        self.body_id = int(model.site_bodyid[lidar_id])
        self.site_pos = model.site_pos[lidar_id].copy()
        self.site_quat = model.site_quat[lidar_id].copy()  # MuJoCo wxyz
        self._publish_static_lidar_tf()
        self.node.create_timer(1.0 / rate_hz, self._publish_latest)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = Thread(target=self.executor.spin, name='ros-lidar-publisher', daemon=True)
        self.thread.start()

    def submit(self, scan, data):
        """Replace pending data with the latest scan; never wait for the ROS thread."""
        if scan is None:
            return
        item = (np.asarray(scan['lidar'], dtype=np.float32).copy(),
                None if scan['map'] is None else np.asarray(scan['map'], dtype=np.float32).copy(),
                data.xpos[self.body_id].copy(), data.xquat[self.body_id].copy())
        try:
            self.latest.get_nowait()
        except Empty:
            pass
        try:
            self.latest.put_nowait(item)
        except Exception:
            pass

    def _cloud(self, points, frame, stamp):
        msg = self.PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        arr = np.ascontiguousarray(points, dtype='<f4').reshape((-1, 3))
        msg.height = 1
        msg.width = int(len(arr))
        msg.fields = self.fields
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.data = arr.tobytes()
        msg.is_dense = True
        return msg

    @staticmethod
    def _set_transform(transform, position, quat_wxyz):
        transform.translation.x = float(position[0])
        transform.translation.y = float(position[1])
        transform.translation.z = float(position[2])
        transform.rotation.x = float(quat_wxyz[1])
        transform.rotation.y = float(quat_wxyz[2])
        transform.rotation.z = float(quat_wxyz[3])
        transform.rotation.w = float(quat_wxyz[0])

    def _publish_static_lidar_tf(self):
        tf = self.TransformStamped()
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'lidar_link'
        self._set_transform(tf.transform, self.site_pos, self.site_quat)
        self.static_tf_pub.sendTransform(tf)

    def _publish_latest(self):
        try:
            points, map_points, base_pos, base_quat = self.latest.get_nowait()
        except Empty:
            return
        stamp = self.node.get_clock().now().to_msg()
        self.points_pub.publish(self._cloud(points, 'lidar_link', stamp))
        tf = self.TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'world'
        tf.child_frame_id = 'base_link'
        self._set_transform(tf.transform, base_pos, base_quat)
        self.tf_pub.sendTransform(tf)
        if self.publish_map and map_points is not None:
            self.map_pub.publish(self._cloud(map_points, 'world', stamp))

    def close(self):
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        if self.owns_context and self.rclpy.ok():
            self.rclpy.shutdown()
