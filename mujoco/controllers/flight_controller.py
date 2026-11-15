"""Cascaded position/attitude controller and four-rotor wrench allocation."""
import math
import numpy as np
from .pid import PID

MASS = 1.477
GRAVITY = 9.81
ARM = 0.18  # estimated from visual mesh extent; rotor centres at +/- ARM
KF = 4.0e-5  # estimated; corresponds to 10 N at 500 rad/s
KQ = 8.0e-7  # estimated reaction-torque coefficient
MAX_OMEGA = 500.0
MAX_THRUST = KF * MAX_OMEGA**2
# Motor numbering: front-left, rear-left, rear-right, front-right (CCW viewed from above).
POSITIONS = np.array([[ARM, ARM, 0], [-ARM, ARM, 0], [-ARM, -ARM, 0], [ARM, -ARM, 0]])
SPIN = np.array([1., -1., 1., -1.])

class FlightController:
    def __init__(self, position_target=(0., 0., 1.2), yaw_target=0., roll_target=0., pitch_target=0.):
        self.position_target = np.array(position_target, dtype=float)
        self.yaw_target = float(yaw_target)
        self.roll_target = float(roll_target)
        self.pitch_target = float(pitch_target)
        self.position_pid = [PID(2.2, 0.15, 1.8, 3.0, 1.5) for _ in range(3)]
        # Existing Gazebo attitude gains: roll/pitch P=10, D=5; yaw P=2, D=1.
        self.attitude_kp = np.array([10., 10., 2.])
        self.rate_kd = np.array([5.0, 5.0, 1.0])
        self.max_tilt = 0.45
        self.mix = np.column_stack((np.ones(4), POSITIONS[:, 1], -POSITIONS[:, 0], SPIN * KQ / KF))
        self.inertia = np.array([0.1152, 0.1152, 0.218])

    @staticmethod
    def _quat_error(q, desired):
        # MuJoCo wxyz. q_err = desired * conjugate(current); use shortest arc.
        w,x,y,z = q
        a,b,c,d = desired
        qi = np.array([w*a+x*b+y*c+z*d, -a*x+b*w+c*z-d*y,
                       -a*y-b*z+c*w+d*x, -a*z+b*y-c*x+d*w])
        if qi[0] < 0: qi = -qi
        return 2.0 * qi[1:]

    @staticmethod
    def _desired_quat(roll, pitch, yaw):
        cr,sr=math.cos(roll/2),math.sin(roll/2); cp,sp=math.cos(pitch/2),math.sin(pitch/2); cy,sy=math.cos(yaw/2),math.sin(yaw/2)
        return np.array([cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy])

    def update(self, position, velocity, quat_wxyz, body_rates, dt):
        pos=np.asarray(position); vel=np.asarray(velocity); rates=np.asarray(body_rates)
        accel=np.array([self.position_pid[i].update(self.position_target[i], pos[i], dt) for i in range(3)])
        accel=np.clip(accel, [-3,-3,-3], [3,3,3])
        roll_cmd=np.clip(self.roll_target+(accel[0]*math.sin(self.yaw_target)-accel[1]*math.cos(self.yaw_target))/GRAVITY, -self.max_tilt, self.max_tilt)
        pitch_cmd=np.clip(self.pitch_target+(accel[0]*math.cos(self.yaw_target)+accel[1]*math.sin(self.yaw_target))/GRAVITY, -self.max_tilt, self.max_tilt)
        qdes=self._desired_quat(roll_cmd,pitch_cmd,self.yaw_target)
        err=self._quat_error(quat_wxyz,qdes)
        torque=self.inertia*(self.attitude_kp*err-self.rate_kd*rates)
        collective=MASS*(GRAVITY+accel[2])/max(0.7, math.cos(roll_cmd)*math.cos(pitch_cmd))
        wrench=np.array([np.clip(collective,0,4*MAX_THRUST), *torque])
        thrust=np.linalg.solve(self.mix.T,wrench)
        thrust=np.clip(thrust,0,MAX_THRUST)
        omega=np.sqrt(thrust/KF)
        return omega, thrust, wrench
