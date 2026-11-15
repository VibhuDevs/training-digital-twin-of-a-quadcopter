from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulation.drone_sim import DroneSimulation
s=DroneSimulation()
assert len(s.sites)==4
assert s.model.nsensor==5
print(f'Model loaded: nq={s.model.nq}, nv={s.model.nv}, four rotor sites and five sensors')
