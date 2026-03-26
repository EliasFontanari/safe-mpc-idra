import numpy as np
import glob
import sys

if sys.argv[1] == 'acados':
    files = sorted(glob.glob('data_results/verification_results_N_CIS_acados_start*.npy'))
    results = np.concatenate([np.load(f) for f in files], axis=0)
    np.save('data_results/verification_results_N_CIS_acados.npy', results)
    print(f'Successful states = {np.sum(results.any(axis=1))} / {results.shape[0]}')
elif sys.argv[1] == 'ipopt':
    files = sorted(glob.glob('data_results/verification_results_N_stepCIS_start*.npy'))
    results = np.concatenate([np.load(f) for f in files], axis=0)
    np.save('data_results/verification_results_N_stepCIS.npy', results)
    print(f'Successful states = {np.sum(results.any(axis=1))} / {results.shape[0]}')
