from pathlib import Path
import sys
import mujoco, numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulation.drone_sim import DroneSimulation
from controllers.flight_controller import MAX_THRUST, POSITIONS, SPIN, KQ, KF
s=DroneSimulation(); rotation=s.data.xmat[s.body].reshape(3,3)
for i,sid in enumerate(s.sites):
 point=s.data.site_xpos[sid].copy(); force=rotation@np.array([0.,0.,MAX_THRUST]); couple=rotation@np.array([0.,0.,SPIN[i]*(KQ/KF)*MAX_THRUST])
 applied=np.zeros(s.model.nv); mujoco.mj_applyFT(s.model,s.data,force,couple,point,s.body,applied)
 lever=np.cross(POSITIONS[i],np.array([0.,0.,MAX_THRUST]))
 expected=lever+np.array([0.,0.,SPIN[i]*(KQ/KF)*MAX_THRUST])
 np.testing.assert_allclose(applied[-3:],expected,atol=1e-7)
 assert force[2]>0
print('All four independent motors produce upward thrust and expected signed roll, pitch, and reaction yaw moments.')
