import mujoco
import mujoco.viewer

# Load URDF
model = mujoco.MjModel.from_xml_path(
    "/home/vibhu/sim/sjtu_drone/sjtu_drone_description/urdf/sjtu_drone.urdf.xml"
)

# Create simulation data
data = mujoco.MjData(model)

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
