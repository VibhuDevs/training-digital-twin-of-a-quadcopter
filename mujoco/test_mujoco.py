import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_string("""
<mujoco>
    <worldbody>
        <light pos="0 0 3"/>

        <geom name="floor"
              type="plane"
              size="5 5 0.1"
              rgba="0.8 0.9 0.8 1"/>

        <body pos="0 0 1">
            <freejoint/>

            <geom type="box"
                  size="0.1 0.1 0.1"
                  mass="1"/>
        </body>
    </worldbody>
</mujoco>
""")

data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
