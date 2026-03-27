import numpy as np
import matplotlib.pyplot as plt

solver = 'ipopt' # choose between 'acados' and 'ipopt' data

# load tested initial conditions
states_to_verify = np.load('data_results/sampled_states.npy')

# load data from acados or ipopt
if solver == 'acados':
    results_N_stepCIS = np.load('data_results/verification_results_N_CIS_acados.npy')
    results_CIS = np.load('data_results/verification_results_CIS.npy')
elif solver == 'ipopt':
    results_N_stepCIS = np.load('data_results/verification_results_N_stepCIS.npy')
    results_CIS = np.load('data_results/verification_results_CIS.npy')

print(f'Successful states for CIS: {np.sum(results_CIS)} / {results_CIS.shape[0]}')
print(f'Successful states for N-step CIS (at least one problem feasible for horizons in 1-45): {np.sum(results_N_stepCIS.any(axis=1))} / {results_N_stepCIS.shape[0]}')

# find inexes at which CIS is not feasible
infeasible_indices = np.where(results_CIS == False)[0]
print(f'Indices that are not safe for CIS: {infeasible_indices}')

# find indices at which N-step CIS is not feasible for any of the tested horizons
infeasible_N_step_indices = np.where(results_N_stepCIS.any(axis=1)==False)[0]
print(f'Indices that are not safe for N-step CIS for any of the tested horizons: {infeasible_N_step_indices}')

print('\n\n\n')

# at CIS infeasible states, find at which horizons N-step CIS becomes feasible, if it is the case
for idx in infeasible_indices:
    string_to_print = f'State {states_to_verify[idx]}:  is Unsafe for CIS\n'
    for horizon in range(results_N_stepCIS[idx,:].shape[0]):
        string_to_print += f' | Horizon {horizon+1}: {"Safe" if results_N_stepCIS[idx,horizon] else "Unsafe"}\n'  
    print(string_to_print) 


