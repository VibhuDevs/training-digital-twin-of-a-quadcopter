from pathlib import Path
import argparse, csv, sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulation.drone_sim import DroneSimulation
p=argparse.ArgumentParser(); p.add_argument('--duration',type=float,default=8.0); p.add_argument('--target',type=float,default=1.2); p.add_argument('--csv',default=str(Path(__file__).resolve().parents[1]/'hover_log.csv')); a=p.parse_args()
s=DroneSimulation(); s.controller.position_target[2]=a.target; rows=[]; next_log=0
for _ in range(round(a.duration/s.model.opt.timestep)):
 state,omega,thrust=s.step()
 if s.data.time>=next_log:
  rows.append([s.data.time,*s.data.qpos[:3],*s.data.qpos[3:7],*omega]); next_log+=0.02
with open(a.csv,'w',newline='') as f:
 w=csv.writer(f); w.writerow(['time','x','y','z','qw','qx','qy','qz','motor1_rad_s','motor2_rad_s','motor3_rad_s','motor4_rad_s']); w.writerows(rows)
z=np.array([r[3] for r in rows]); q=np.array([r[4:8] for r in rows]); tilt=np.arccos(np.clip(1-2*(q[:,2]**2+q[:,1]**2),-1,1))
print(f'Hover run {a.duration:.1f}s: z start={z[0]:.3f} m, end={z[-1]:.3f} m, range=[{z.min():.3f},{z.max():.3f}] m, max tilt={tilt.max():.3f} rad; log={a.csv}')
if not (abs(z[-1]-a.target)<0.18 and z.min()>0.1 and tilt.max()<0.35): raise SystemExit('Hover criteria failed')
