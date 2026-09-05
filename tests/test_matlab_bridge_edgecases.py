import os
import sys
import tempfile
import scipy.io

# Ensure utilities path is importable
HERE = os.path.dirname(os.path.dirname(__file__))
UTILS_PATH = os.path.join(HERE, 'python', 'utilities')
sys.path.insert(0, UTILS_PATH)

import matlab_bridge


def test_empty_objects():
    with tempfile.TemporaryDirectory() as td:
        mat_path = os.path.join(td, 'empty.mat')
        matlab_bridge.save_objects_to_mat([], mat_path)
        data = scipy.io.loadmat(mat_path, squeeze_me=True)
        # scipy may store empty as array([]), ensure objects present
        assert 'objects' in data


def test_missing_fields():
    objs = [
        {'id': 1, 'x': 0},
        {'id': 2, 'y': 1},
    ]
    with tempfile.TemporaryDirectory() as td:
        mat_path = os.path.join(td, 'missing.mat')
        matlab_bridge.save_objects_to_mat(objs, mat_path)
        data = scipy.io.loadmat(mat_path, squeeze_me=True)
        assert 'objects' in data


def test_non_primitive_values():
    objs = [
        {'id': 1, 'type': 'car', 'meta': {'color': 'red'}},
    ]
    with tempfile.TemporaryDirectory() as td:
        mat_path = os.path.join(td, 'nonprim.mat')
        # Should not raise — nested dicts will be saved as Python objects by scipy
        matlab_bridge.save_objects_to_mat(objs, mat_path)
        data = scipy.io.loadmat(mat_path, squeeze_me=True)
        assert 'objects' in data
