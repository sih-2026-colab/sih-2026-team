"""
MATLAB Bridge Utilities

Functions to save/load objects to/from MAT-files for exchange between Python and MATLAB.
"""
import json
import os
from typing import List, Dict, Any

import scipy.io


def save_objects_to_mat(objects: List[Dict[str, Any]], mat_path: str = 'input_objects.mat') -> None:
    """Save a list of dict objects to a MATLAB .mat file as a struct array.

    Args:
        objects: list of dicts with primitive types
        mat_path: output .mat file path
    """
    # Convert list of dicts to a dict of lists for scipy.io
    if not objects:
        scipy.io.savemat(mat_path, {'objects': []})
        return

    # Determine all keys
    keys = set()
    for obj in objects:
        keys.update(obj.keys())
    keys = sorted(keys)

    # Build a MATLAB struct array as a list of dicts
    mat_struct = []
    for obj in objects:
        # Convert None values to MATLAB-friendly defaults (empty string)
        entry = {}
        for k in keys:
            v = obj.get(k, None)
            if v is None:
                # Use empty string so scipy can write a value for mixed-type structs
                v = ''
            entry[k] = v
        mat_struct.append(entry)

    # scipy expects dict mapping names to arrays; using object arrays
    scipy.io.savemat(mat_path, {'objects': mat_struct})


def load_mat_struct_array(mat_path: str, var_name: str = 'detections') -> List[Dict[str, Any]]:
    """Load a struct array from a .mat file and convert to list of dicts.

    Args:
        mat_path: path to the .mat file
        var_name: variable name inside the mat file

    Returns:
        List of dictionaries representing the struct array
    """
    if not os.path.exists(mat_path):
        return []

    data = scipy.io.loadmat(mat_path, squeeze_me=True, struct_as_record=False)
    if var_name not in data:
        return []

    arr = data[var_name]
    # If empty or scalar
    if isinstance(arr, (int, float)) or arr is None:
        return []

    # If single struct
    results = []
    try:
        for item in arr:
            d = {}
            for field in item._fieldnames:
                val = getattr(item, field)
                d[field] = val.tolist() if hasattr(val, 'tolist') else val
            results.append(d)
    except Exception:
        # Fallback for non-numpy structured arrays
        try:
            fields = arr.dtype.names
            for rec in arr:
                d = {f: rec[f].item() if hasattr(rec[f], 'item') else rec[f] for f in fields}
                results.append(d)
        except Exception:
            # Unknown structure
            return []

    return results


def save_json(objects: List[Dict[str, Any]], json_path: str) -> None:
    """Save objects as JSON for readability and cross-checking."""
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(objects, f, indent=2)


if __name__ == '__main__':
    # Quick smoke test
    objs = [
        {'id': 1, 'type': 'car', 'x': 20, 'y': 2, 'speed': 8},
        {'id': 2, 'type': 'pedestrian', 'x': 14, 'y': -1, 'speed': 1.2},
    ]
    save_objects_to_mat(objs, 'input_objects.mat')
    print('Wrote input_objects.mat')
