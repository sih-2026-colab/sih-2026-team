"""
Demo runner showing how Python can write objects for MATLAB, optionally trigger MATLAB
and read back results. This is a lightweight integration helper and assumes MATLAB is
installed and available on the system path as `matlab` (Windows: `matlab.exe`).
"""
import os
import subprocess
import sys
from typing import List, Dict

from matlab_bridge import save_objects_to_mat, load_mat_struct_array, save_json


def main():
    # Example objects (same format used across pipeline)
    objects: List[Dict] = [
        {"id": 1, "type": "car", "x": 20, "y": 2, "speed": 8},
        {"id": 2, "type": "pedestrian", "x": 14, "y": -1, "speed": 1.2},
        {"id": 3, "type": "cow", "x": 25, "y": 1, "speed": 0.8},
    ]

    # Save objects for MATLAB
    print('Saving objects to input_objects.mat')
    save_objects_to_mat(objects, 'input_objects.mat')
    save_json(objects, 'input_objects.json')

    # Optionally call MATLAB to run integration script
    matlab_cmd = os.environ.get('MATLAB_CMD', 'matlab -batch "run(\'matlab/main/run_integration.m\')"')
    print('MATLAB command to run:', matlab_cmd)
    run_matlab = os.environ.get('RUN_MATLAB', '0')

    if run_matlab == '1':
        try:
            print('Invoking MATLAB...')
            subprocess.check_call(matlab_cmd, shell=True)
        except subprocess.CalledProcessError as e:
            print('MATLAB execution failed:', e)
            print('Check MATLAB installation and PATH or set MATLAB_CMD to a custom command.')
            sys.exit(1)
    else:
        print('Skipping MATLAB run. Set environment variable RUN_MATLAB=1 to execute MATLAB.')

    # Read outputs (planning_output.mat)
        # Prefer JSON outputs if MATLAB wrote them
        if os.path.exists('planning_output.json'):
            print('Found planning_output.json:')
            print(open('planning_output.json','r',encoding='utf-8').read())
        else:
            results = load_mat_struct_array('planning_output.mat', var_name='plans')
            if not results:
                print('No planning_output.mat found or empty. If you ran MATLAB, check its current folder.')
            else:
                print('Planning results:')
                for r in results:
                    print(r)


if __name__ == '__main__':
    main()
