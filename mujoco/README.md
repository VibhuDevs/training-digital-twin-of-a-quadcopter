# SJTU Drone in MuJoCo

Independent MuJoCo port; ROS 2/Gazebo source files are unchanged. Activate the existing venv with `source ~/sim/mujoco/mujoco_env/bin/activate`. Run `python ~/sim/mujoco/scripts/test_model.py`, `python ~/sim/mujoco/scripts/test_motors.py`, `python ~/sim/mujoco/scripts/hover_test.py`, `python ~/sim/mujoco/scripts/command_test.py`, or `python ~/sim/mujoco/scripts/run_viewer.py`. `hover_test.py` records a CSV beside this README by default (or use `--csv /path/to/log.csv`).

## Reused parameters and adaptations

The original URDF/SDF specify mass 1.477 kg and diagonal inertia `(0.1152, 0.1152, 0.218) kg m^2` about the body origin/COM. The mesh is 0.52 m wide. Existing controller gains are roll/pitch P=10, D=5, limit 0.5 rad; yaw P=2, D=1, limit 1.5; horizontal velocity P=5, D=2.3; vertical velocity P=5, I=0, D=1; horizontal position P=1.1; vertical position P=1, D=0.2. These set the starting reference; the MuJoCo controller uses quaternion attitude error, the source roll/pitch and yaw derivative gains against body gyro rates, position/velocity feedback and bounded integral action. Gazebo `UpdateDynamics` computed a net thrust and body torque and called `AddRelativeForce`/`AddRelativeTorque`; it did not simulate individual rotors. Its `maxForce=30 N` is a total force cap, not a rotor coefficient.

No rotor centers, motor numbers/directions, thrust coefficients or reaction torque coefficients were found. Estimated rotor sites use a symmetric X layout at `(±0.18, ±0.18, 0) m`, compatible with the mesh span. Motor order is front-left, rear-left, rear-right, front-right; spin signs viewed from above are `+,-,+,-`. Estimated `kf=4e-5 N/(rad/s)^2`, `kq=8e-7 N m/(rad/s)^2`, `omega_max=500 rad/s` give 10 N max thrust per rotor and a 40 N total cap. These estimates are explicit placeholders to make an independently actuated model possible; replace them with measured hardware/Gazebo parameters when found.

## Motor equations

For each motor, `T_i = kf * omega_i^2` and `Q_i = spin_i * kq * omega_i^2`. The force acts along body +Z at the site. The resulting moment is `r_i × (0,0,T_i) + (0,0,Q_i)`. The allocator solves the four by four wrench matrix for collective thrust, roll, pitch and yaw, then clamps thrust to `[0,10] N` before converting back to angular speed.

## Commands and limitations

Edit `position_target` and `yaw_target` in `DroneSimulation`/`FlightController` to change position and heading. The XML uses the original STL as a collision-free visual and a separate simple body collision proxy, with original mass/inertia assigned to the free body. MuJoCo's IMU sensors and frame sensors provide gyro, accelerometer, pose and velocity. The starter URDF conversion is not used as a dynamics model because its geometry-only body lacks a ground-contact-ready collision shape, rotor physics and sensors. Rotor drag, motor lag, battery limits and sensor noise are not modeled. The motor estimates and rotor center estimate require physical calibration. The bundled model, motor, hover, and command-response checks have been run locally; the LiDAR-specific results are recorded below.

## Warehouse environment

`models/warehouse/warehouse_drone.xml` is a combined MJCF model converted from the supplied Gazebo SDF world. It contains the 24 x 16 m floor, four boundary wall sections and south entrance, all three rack rows (uprights and three shelf levels), two pallets, warehouse lighting and the SJTU drone with its original visual mesh, inertial values, rotor sites and sensors. Boxes use their SDF dimensions and poses; matching SDF visual/collision box pairs become one MuJoCo geom that both renders and collides. The ceiling light fixtures remain visual only as in the SDF. Gazebo system plugins (physics, contact, sensors, command and scene broadcaster) are not portable and are not copied; MuJoCo supplies its own physics, contact and sensor handling. The MJCF timestep is 1 ms to match the SDF.

The drone starts at `(0, 2.2, 0.25) m`, in the aisle between the middle and north rack rows, and targets `(0, 2.2, 1.2) m`. The old center spawn would be underneath a rack shelf. The interactive viewer loads this combined environment. Use WASD to change horizontal position, R/F for altitude, Q/E for yaw, and Home to reset the target. The clear aisle is narrow, so avoid steering into the rack rows or walls.

Warehouse validation rerun: the combined model loaded with four rotor sites and five sensors; signed motor moment checks passed; an 8-second hover ended at 1.235 m for the 1.2 m target (logged to `/tmp/warehouse_hover.csv`); the movement/return command sequence ended at 1.226 m with 0.142 rad tilt.


## Live 3D LiDAR and RViz2

`simulation/lidar_simulator.py` uses `mujoco.mj_multiRay` against the same model/data used by the drone. Defaults are 360° horizontal FOV, -15°..+15° vertical FOV, 0.5° azimuth spacing, 16 channels, 0.1–50 m range, and 10 Hz; change them in `config/lidar_config.yaml`. The rays follow the mounted `lidar_site`, ignore the drone body, and include collision-enabled geometry groups. Local-frame XYZ is published as `/drone/lidar/points` (`sensor_msgs/msg/PointCloud2`, frame `lidar_link`). TF is `world -> base_link -> lidar_link`, with MuJoCo's Z-up axes mapped directly to ROS ENU-style XYZ and WXYZ quaternions reordered to ROS XYZW.

The ROS bridge uses a size-one mailbox and a background executor thread so publishing cannot block physics. To launch the combined simulator with ROS support, open a terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PYTHONPATH:/usr/lib/python3/dist-packages"
source ~/sim/mujoco/mujoco_env/bin/activate
python ~/sim/mujoco/scripts/run_viewer.py
```

In another terminal, source ROS, check the live topic rate, and open RViz2 with the included configuration:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /drone/lidar/points
rviz2 -d ~/sim/mujoco/config/drone_lidar.rviz
```

Alternatively, in a ROS-sourced terminal run `python3 ~/sim/mujoco/launch/lidar_visualization.launch.py` to start the included launch description directly. The point topic uses best-effort sensor QoS. RViz2 fixed frame is `world`. Optional map accumulation is bounded by `max_accumulated_points`; set `accumulate_map: true` in the config or press **M** at runtime to enable it. It publishes `/drone/lidar/map_points` in `world`; press **C** to clear the map. The RViz map display is disabled by default and can be enabled from the Displays panel.

LiDAR validation commands: `python scripts/test_lidar.py` checks site pose, translation/rotation tracking, real wall and shelf hits, finite XYZ coordinates, and the 10 Hz scan gate. With ROS sourced and `PYTHONPATH` extended as above, `python scripts/test_ros_lidar.py` checks PointCloud2 content/frequency and both TF links. ROS and RViz delivery still require a sourced ROS 2 Jazzy environment and a functioning desktop display.
