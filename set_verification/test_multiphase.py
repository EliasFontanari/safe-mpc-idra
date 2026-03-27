import time
import pickle
import numpy as np
from tqdm import tqdm
from scipy.stats import qmc
from safe_mpc.parser import Parameters, parse_args
from safe_mpc.env_model import AdamModel
from safe_mpc.utils import  get_controller , get_ocp_acados, randomize_model
from safe_mpc.controller import  BackAndForthNstepController
from safe_mpc.robot_visualizer import RobotVisualizer
from safe_mpc.ocp import InverseKinematicsOCP
import copy
from safe_mpc.cost_definition import *

args = parse_args()
start = args['start_array']
end = args['end_array']
model_name = args['system']
params = Parameters(args,model_name, rti=False)
params.q_margin = args['joint_bounds_margin']
params.collision_margin = args['collision_margin']
params.build = args['build']
params.act = args['activation']
params.solver_type = 'SQP'
params.N=1
params.N = args['horizon']
params.noise_mass = args['noise']
params.noise_inertia = args['noise']
params.noise_cm = args['noise']

model = AdamModel(params)

# cost = TrackingMovingCircleNLS(model,params.Q_weight,params.R_weight)
#cost = Tracking8NLS(model,params.Q_weight,params.R_weight)
# cost = ZeroCost(model)
# cost = ReachTargetNLS(model,params.Q_weight,params.R_weight)

ocp_name = 'htwa'
params.cont_name = args['controller']

build_controllers = args['build']

ocp_with_net = BackAndForthNstepController(model, k_backward = 1)
ocp_with_net.build_controller(build = build_controllers)

n_horizon_to_test = 44
states_to_verify = np.load('data_results/sampled_states.npy')[start:end]
results = np.zeros((states_to_verify.shape[0],n_horizon_to_test), dtype=bool)

# to have minimum number of compilations, iterate on the horizons, and for each horizon iterate on the states
progress_bar = tqdm(total=n_horizon_to_test, desc=f'Testing initial conditions, alpha {ocp_with_net.model.params.alpha}')
for i in range(n_horizon_to_test,0,-1):
    ocp_with_net = BackAndForthNstepController(model, k_backward = i)
    ocp_with_net.build_controller(build = build_controllers)
    for j in range(states_to_verify.shape[0]):
        x_k = states_to_verify[j]
        status = ocp_with_net.solve(x_k)
        if (status == 0 or status == 2) and ocp_with_net.checkGuess():
            results[j, i] = True
        else:
            results[j, i] = False

        print(f"State {j}_{x_k}: {'Safe' if results[j, i] else 'Unsafe'}")
    progress_bar.update(1)
for i in range(states_to_verify.shape[0]):
    if (results[i,:] == False).all():
            print(f'State {i}_{states_to_verify[i]}:  is Unsafe for all horizons')

np.save(f'data_results/verification_results_back_and_forth_within_N_CIS_acados_{start}_{end}.npy', results)
print(f'succesful states = {np.sum(results.any(axis=1))} / {results.shape[0]}')
    
