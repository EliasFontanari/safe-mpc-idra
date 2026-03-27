import numpy as np
import glob
import sys
import re


def _chunk_sort_key(path):
    """Sort files by numeric start/end indices in the filename."""
    m = re.search(r'_start(\d+)_end(\d+)\.npy$', path)
    if m is None:
        return (float('inf'), float('inf'), path)
    return (int(m.group(1)), int(m.group(2)), path)

if sys.argv[1] == 'acados':
    files = sorted(
        glob.glob('data_results/verification_results_N_CIS_acados_start*.npy'),
        key=_chunk_sort_key,
    )
    results = np.concatenate([np.load(f) for f in files], axis=0)
    np.save('data_results/verification_results_N_CIS_acados.npy', results)
    print(f'Successful states = {np.sum(results.any(axis=1))} / {results.shape[0]}')
elif sys.argv[1] == 'ipopt':
    files = sorted(
        glob.glob('data_results/verification_results_N_stepCIS_start*.npy'),
        key=_chunk_sort_key,
    )
    results = np.concatenate([np.load(f) for f in files], axis=0)
    np.save('data_results/verification_results_N_stepCIS.npy', results)
    print(f'Successful states = {np.sum(results.any(axis=1))} / {results.shape[0]}')
