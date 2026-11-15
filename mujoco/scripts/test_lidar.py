"""Repeatable MuJoCo LiDAR pose, collision-ray, and rate checks."""
from pathlib import Path
import sys
import numpy as np
import mujoco
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation.drone_sim import DroneSimulation
from simulation.lidar_simulator import LidarSimulator

sim = DroneSimulation()
lidar = LidarSimulator(sim.model, sim.data)
origin, rotation = lidar.get_lidar_pose()
np.testing.assert_allclose(origin, [0., 2.2, 0.415], atol=1e-6)
assert rotation.shape == (3, 3) and lidar.nray == 11520
cloud = lidar.perform_scan()
assert cloud.ndim == 2 and cloud.shape[1] == 3 and len(cloud) > 1000
assert np.isfinite(cloud).all()
hit_names = {sim.model.geom(int(i)).name for i in lidar._hit_ids if i >= 0}
assert any('wall' in name for name in hit_names), f'no wall hits: {hit_names}'
assert any('shelf' in name for name in hit_names), f'no rack hits: {hit_names}'

# Check that the sensor site follows base translation and rotation.
sim.data.qpos[:3] = [1.0, -1.0, 0.5]
sim.data.qpos[3:7] = [np.sqrt(0.5), 0., 0., np.sqrt(0.5)]
mujoco.mj_forward(sim.model, sim.data)
origin2, rotation2 = lidar.get_lidar_pose()
np.testing.assert_allclose(origin2, [1., -1., 0.665], atol=1e-6)
np.testing.assert_allclose(rotation2 @ [1., 0., 0.], [0., 1., 0.], atol=1e-6)
assert lidar.scan_if_due(0.0) is not None
assert lidar.scan_if_due(0.05) is None
assert lidar.scan_if_due(0.1) is not None
print(f'LiDAR checks passed: {len(cloud)} environment hits from {lidar.nray} rays; wall and shelf hits; pose and 10 Hz gate verified.')
