"""ROS integration check; run after sourcing ROS 2 Jazzy and activating the venv."""
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                      ReliabilityPolicy, DurabilityPolicy)
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from simulation.drone_sim import DroneSimulation
from simulation.lidar_simulator import LidarSimulator
from scripts.lidar_ros_publisher import LidarRosBridge

sim = DroneSimulation()
lidar = LidarSimulator(sim.model, sim.data)
bridge = LidarRosBridge(sim.model, lidar.lidar_id, rate_hz=30.0)
probe = rclpy.create_node('mujoco_lidar_test_probe')
clouds, dynamic_tf, static_tf = [], [], []
probe.create_subscription(PointCloud2, '/drone/lidar/points', clouds.append, qos_profile_sensor_data)
probe.create_subscription(TFMessage, '/tf', lambda msg: dynamic_tf.extend(msg.transforms), 10)
static_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
probe.create_subscription(TFMessage, '/tf_static', lambda msg: static_tf.extend(msg.transforms), static_qos)
executor = SingleThreadedExecutor()
executor.add_node(probe)
start = time.monotonic()
try:
    for _ in range(6):
        cloud = lidar.perform_scan()
        bridge.submit({'lidar': cloud, 'map': None}, sim.data)
        deadline = time.monotonic() + 0.12
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
    assert len(clouds) >= 5, f'expected ~10 Hz PointCloud2 stream, got {len(clouds)} messages'
    msg = clouds[-1]
    assert msg.header.frame_id == 'lidar_link'
    assert msg.width > 1000 and msg.height == 1
    assert msg.point_step == 12 and msg.row_step == msg.width * 12
    assert len(msg.data) == msg.width * msg.point_step
    assert [field.name for field in msg.fields] == ['x', 'y', 'z']
    assert any(t.header.frame_id == 'world' and t.child_frame_id == 'base_link' for t in dynamic_tf)
    assert any(t.header.frame_id == 'base_link' and t.child_frame_id == 'lidar_link' for t in static_tf)
    elapsed = time.monotonic() - start
    print(f'ROS LiDAR passed: {len(clouds)} clouds in {elapsed:.2f}s, '
          f'{msg.width} points, frame={msg.header.frame_id}, TF tree complete.')
finally:
    executor.remove_node(probe)
    probe.destroy_node()
    bridge.close()
