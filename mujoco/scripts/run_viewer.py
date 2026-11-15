"""Interactive warehouse drone, LiDAR raycaster, and asynchronous ROS 2 publisher."""
from pathlib import Path
import sys
import time

import mujoco
import mujoco.viewer
import glfw

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulation.drone_sim import DroneSimulation
from simulation.lidar_simulator import LidarSimulator
from scripts.lidar_ros_publisher import LidarRosBridge

sim = DroneSimulation()
lidar = LidarSimulator(sim.model, sim.data)
try:
    # Publish at the scan cadence. The bridge's worker thread and size-one mailbox
    # keep ROS/RViz transport out of the physics loop.
    ros_bridge = LidarRosBridge(
        sim.model,
        lidar.lidar_id,
        rate_hz=max(30.0, float(lidar.config['scan_frequency_hz'])),
        publish_map=True,
    )
    print('[INFO] Publishing /drone/lidar/points (frame lidar_link).')
except Exception as exc:
    ros_bridge = None
    print(f'[ERROR] ROS 2 publisher did not start: {exc}')
    print('Source ROS 2 Jazzy and expose system Python packages before launching.')

STEP = 0.2

def on_key(key):
    if key == glfw.KEY_W:
        sim.controller.position_target[0] += STEP
    elif key == glfw.KEY_S:
        sim.controller.position_target[0] -= STEP
    elif key == glfw.KEY_A:
        sim.controller.position_target[1] += STEP
    elif key == glfw.KEY_D:
        sim.controller.position_target[1] -= STEP
    elif key == glfw.KEY_R:
        sim.controller.position_target[2] += STEP
    elif key == glfw.KEY_F:
        sim.controller.position_target[2] = max(0.35, sim.controller.position_target[2] - STEP)
    elif key == glfw.KEY_Q:
        sim.controller.yaw_target += 0.2
    elif key == glfw.KEY_E:
        sim.controller.yaw_target -= 0.2
    elif key == glfw.KEY_HOME:
        sim.controller.position_target[:] = [0.0, 2.2, 1.2]
        sim.controller.yaw_target = 0.0
    elif key == glfw.KEY_M:
        lidar.accumulate_map = not lidar.accumulate_map
        if not lidar.accumulate_map:
            lidar.clear_accumulated()
        print(f"[LIDAR] accumulated map {'enabled' if lidar.accumulate_map else 'disabled'}")
    elif key == glfw.KEY_C:
        lidar.clear_accumulated()
        print('[LIDAR] accumulated map cleared')
    print(f'target xyz={sim.controller.position_target.round(2)}, yaw={sim.controller.yaw_target:.2f} rad')

print('Controls: W/S forward/back, A/D left/right, R/F up/down, Q/E yaw, HOME reset, M map toggle, C clear map.')
try:
    with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=on_key) as viewer:
        while viewer.is_running():
            start = time.time()
            sim.step()
            scan = lidar.scan_if_due(sim.data.time)
            if scan is not None:
                if ros_bridge is not None:
                    ros_bridge.submit(scan, sim.data)
                print(f"[LIDAR] scan={lidar.scan_count}, points={len(scan['lidar'])}")
            viewer.sync()
            time.sleep(max(0.0, sim.model.opt.timestep - (time.time() - start)))
finally:
    if ros_bridge is not None:
        ros_bridge.close()
print('[INFO] Simulation stopped.')
