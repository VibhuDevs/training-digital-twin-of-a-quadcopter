"""Environment-derived 3D LiDAR using MuJoCo's multi-ray intersection API."""
from pathlib import Path
import ast
import numpy as np
import mujoco

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config/lidar_config.yaml'


def _read_simple_yaml(path):
    """Read the scalar/list-only configuration without requiring PyYAML."""
    result = {}
    for line in Path(path).read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if not line or ':' not in line:
            continue
        key, raw = (part.strip() for part in line.split(':', 1))
        lowered = raw.lower()
        if lowered in ('true', 'false'):
            value = lowered == 'true'
        else:
            try:
                value = ast.literal_eval(raw)
            except (ValueError, SyntaxError):
                value = raw
        result[key] = value
    return result


class LidarSimulator:
    """Ray-cast scanner bound to an existing MuJoCo model and data instance."""
    def __init__(self, model, data, config_path=DEFAULT_CONFIG):
        self.model = model
        self.data = data
        self.config = _read_simple_yaml(config_path)
        self.lidar_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, 'lidar_site'
        )
        if self.lidar_id < 0:
            raise ValueError('lidar_site not found in loaded MJCF')
        self.body_id = int(model.site_bodyid[self.lidar_id])
        self.scan_period = 1.0 / float(self.config['scan_frequency_hz'])
        self.next_scan_time = 0.0
        self.accumulate_map = bool(self.config.get('accumulate_map', False))
        self.max_accumulated_points = int(self.config.get('max_accumulated_points', 200000))
        self.accumulated_world = np.empty((0, 3), dtype=np.float32)
        self.scan_count = 0

        h_fov = float(self.config['horizontal_fov_deg'])
        h_res = float(self.config['horizontal_resolution_deg'])
        v_min, v_max = map(float, self.config['vertical_fov_deg'])
        channels = int(self.config['vertical_channels'])
        azimuth = np.deg2rad(np.arange(-h_fov / 2.0, h_fov / 2.0, h_res))
        elevation = np.deg2rad(np.linspace(v_min, v_max, channels))
        az, el = np.meshgrid(azimuth, elevation)
        self.directions_lidar = np.column_stack((
            np.cos(el).ravel() * np.cos(az).ravel(),
            np.cos(el).ravel() * np.sin(az).ravel(),
            np.sin(el).ravel(),
        )).astype(np.float64, copy=False)
        self.nray = self.directions_lidar.shape[0]
        self.max_range = float(self.config['max_range_m'])
        self.min_range = float(self.config['min_range_m'])
        # mj_multiRay filters geom groups, not contact masks. Include only groups
        # containing collision-enabled geoms; bodyexclude removes the drone.
        self.geom_groups = np.zeros(6, dtype=np.uint8)
        for geom_id in range(model.ngeom):
            if model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]:
                group = int(model.geom_group[geom_id])
                if 0 <= group < len(self.geom_groups):
                    self.geom_groups[group] = 1
        self._hit_ids = np.full(self.nray, -1, dtype=np.int32)
        self._distances = np.full(self.nray, -1.0, dtype=np.float64)

    def get_lidar_pose(self):
        origin = self.data.site_xpos[self.lidar_id].copy()
        rotation = self.data.site_xmat[self.lidar_id].reshape(3, 3).copy()
        return origin, rotation

    def perform_scan(self):
        """Return valid XYZ hits in LiDAR coordinates as an (N, 3) array."""
        origin, rotation = self.get_lidar_pose()
        directions_world = np.ascontiguousarray(
            (rotation @ self.directions_lidar.T).T, dtype=np.float64
        )
        self._hit_ids.fill(-1)
        self._distances.fill(-1.0)
        mujoco.mj_multiRay(
            self.model, self.data, np.ascontiguousarray(origin),
            directions_world.ravel(), self.geom_groups, True, self.body_id,
            self._hit_ids, self._distances, None, self.nray, self.max_range,
        )
        valid = (
            (self._hit_ids >= 0)
            & (self._distances >= self.min_range)
            & (self._distances <= self.max_range)
            & np.isfinite(self._distances)
        )
        return (self.directions_lidar[valid] * self._distances[valid, None]).astype(np.float32)

    def generate_pointcloud(self, frame='lidar'):
        """Return a scan in LiDAR or world coordinates."""
        points = self.perform_scan()
        if frame == 'lidar':
            return points
        if frame == 'world':
            origin, rotation = self.get_lidar_pose()
            return (points @ rotation.T + origin).astype(np.float32)
        raise ValueError("frame must be 'lidar' or 'world'")

    def clear_accumulated(self):
        self.accumulated_world = np.empty((0, 3), dtype=np.float32)

    def scan_if_due(self, sim_time=None):
        """Run at configured simulation-time frequency; return None between scans."""
        now = float(self.data.time if sim_time is None else sim_time)
        if now + 1e-9 < self.next_scan_time:
            return None
        self.next_scan_time += self.scan_period
        while self.next_scan_time <= now:
            self.next_scan_time += self.scan_period
        origin, rotation = self.get_lidar_pose()
        local = self.perform_scan()
        world = (local @ rotation.T + origin).astype(np.float32)
        self.scan_count += 1
        if self.accumulate_map and len(world):
            self.accumulated_world = np.concatenate((self.accumulated_world, world))
            if len(self.accumulated_world) > self.max_accumulated_points:
                keep = np.linspace(0, len(self.accumulated_world)-1,
                                   self.max_accumulated_points, dtype=np.int64)
                self.accumulated_world = self.accumulated_world[keep]
        return {
            'lidar': local,
            'world': world,
            'map': self.accumulated_world.copy() if self.accumulate_map else None,
            'scan_time': now,
        }
