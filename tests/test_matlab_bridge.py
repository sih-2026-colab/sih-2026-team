import os
import sys
import tempfile
import os
import sys
import tempfile
import scipy.io

# Ensure the python utilities path is importable
HERE = os.path.dirname(os.path.dirname(__file__))
UTILS_PATH = os.path.join(HERE, 'python', 'utilities')
sys.path.insert(0, UTILS_PATH)

import matlab_bridge


def test_mat_roundtrip_simple():
    objs = [
        {'id': 1, 'type': 'car', 'x': 20, 'y': 2, 'speed': 8},
        {'id': 2, 'type': 'pedestrian', 'x': 14, 'y': -1, 'speed': 1.2},
    ]

    with tempfile.TemporaryDirectory() as td:
        mat_path = os.path.join(td, 'input_objects.mat')
        matlab_bridge.save_objects_to_mat(objs, mat_path)

        # load via scipy directly to ensure file exists
        data = scipy.io.loadmat(mat_path, squeeze_me=True)
        assert 'objects' in data

        # Save under 'detections' name and read back with loader
        detections_path = os.path.join(td, 'detections.mat')
        scipy.io.savemat(detections_path, {'detections': data['objects']})
        loaded = matlab_bridge.load_mat_struct_array(detections_path, var_name='detections')

        # Basic checks
        assert isinstance(loaded, list)
        assert len(loaded) == len(objs)
        assert int(loaded[0]['id']) == objs[0]['id']