
"""Apply individual rotor forces at their locations and reaction couples."""

from pathlib import Path

import mujoco
import numpy as np

from controllers.flight_controller import (
    FlightController, POSITIONS, SPIN, KF, KQ
)

from simulation.sensors import read_sensors


MODEL = (
    Path(__file__).resolve().parents[1]
    / 'models/warehouse/warehouse_drone.xml'
)


class DroneSimulation:

    def __init__(self, target=None):

        # =====================================
        # LOAD MUJOCO MODEL
        # =====================================

        self.model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.data = mujoco.MjData(self.model)

        self.controller = FlightController(
            target if target is not None else (0., 2.2, 1.2)
        )

        self.body = self.model.body('base_link').id

        self.sites = [
            self.model.site(f'motor_{i}').id
            for i in range(1, 5)
        ]

        # Initial drone position
        self.data.qpos[:3] = [0., 2.2, 0.25]

        mujoco.mj_forward(self.model, self.data)

        # =====================================
        # LIDAR INITIALIZATION
        # =====================================

        self.lidar_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            "lidar_site"
        )

        if self.lidar_id == -1:
            raise ValueError(
                "lidar_site not found in loaded MJCF: "
                f"{MODEL}"
            )

        print("[INFO] LiDAR site found successfully.")

        print(
            "[INFO] Initial LiDAR position:",
            np.round(self.data.site_xpos[self.lidar_id], 3)
        )

    # =====================================
    # LIDAR POSE
    # =====================================

    def get_lidar_pose(self):

        lidar_origin = self.data.site_xpos[
            self.lidar_id
        ].copy()

        lidar_rotation = self.data.site_xmat[
            self.lidar_id
        ].reshape(3, 3).copy()

        return lidar_origin, lidar_rotation

    # =====================================
    # DRONE PHYSICS AND CONTROL
    # =====================================

    def step(self, control_dt=None):

        dt = (
            self.model.opt.timestep
            if control_dt is None
            else control_dt
        )

        s = read_sensors(self.model, self.data)

        omega, thrust, _ = self.controller.update(
            s['position'],
            s['velocity'],
            s['quat'],
            s['gyro'],
            dt
        )

        self.data.qfrc_applied[:] = 0

        rotation = self.data.xmat[
            self.body
        ].reshape(3, 3)

        for i, (sid, t) in enumerate(
            zip(self.sites, thrust)
        ):

            point = self.data.site_xpos[sid].copy()

            force = rotation @ np.array([0., 0., t])

            couple = rotation @ np.array([
                0.,
                0.,
                SPIN[i] * (KQ / KF) * t
            ])

            mujoco.mj_applyFT(
                self.model,
                self.data,
                force,
                couple,
                point,
                self.body,
                self.data.qfrc_applied
            )

        mujoco.mj_step(self.model, self.data)

        return s, omega, thrust

