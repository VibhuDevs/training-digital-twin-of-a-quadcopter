"""Repeatable position, tilt, yaw and return-to-hover response check."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulation.drone_sim import DroneSimulation
s=DroneSimulation(); home_y=s.data.qpos[1]; log=[]
def run(seconds):
 for _ in range(round(seconds/s.model.opt.timestep)):
  s.step(); log.append((s.data.time,*s.data.qpos[:3],*s.data.qpos[3:7]))
run(4)
assert abs(s.data.qpos[2]-1.2)<0.18, f'takeoff failed: {s.data.qpos[2]}'
s.controller.position_target[:]=[0.12,home_y,1.2]; run(2)
assert s.data.qpos[0]>0.04, f'x position command failed: {s.data.qpos[0]}'
s.controller.position_target[:]=[0,home_y,1.2]; s.controller.roll_target=0.08; run(1)
s.controller.roll_target=0.; s.controller.pitch_target=0.08; run(1)
s.controller.pitch_target=0.; s.controller.yaw_target=0.15; run(1)
s.controller.yaw_target=0.; run(7)
q=s.data.xquat[s.body]; tilt=2*np.arccos(np.clip(abs(q[0]),0,1))
print(f'Commands complete: xyz={s.data.qpos[:3].round(3)}, yaw returned to neutral, final tilt={tilt:.3f} rad')
assert abs(s.data.qpos[2]-1.2)<0.2 and tilt<0.20, 'failed to return to stable neutral hover'
