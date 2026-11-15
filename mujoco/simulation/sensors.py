"""Named access to MuJoCo ground truth and IMU sensor channels."""
import numpy as np

def read_sensors(model, data):
    ids={name:model.sensor(name).id for name in ('position','linear_velocity','orientation','imu_gyro','imu_accel')}
    def read(name):
        sid=ids[name]; adr=model.sensor_adr[sid]; dim=model.sensor_dim[sid]
        return data.sensordata[adr:adr+dim].copy()
    # Use MuJoCo's canonical body pose and free-joint translational velocity for feedback.
    # Frame sensors remain exposed above; direct state avoids ambiguity about sensor reference frames.
    body_id=model.body('base_link').id
    quat=data.xquat[body_id].copy()  # wxyz
    velocity=data.qvel[:3].copy()
    return {'position':read('position'), 'velocity':velocity, 'quat':quat,
            'gyro':read('imu_gyro'), 'accel':read('imu_accel')}
