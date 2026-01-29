from functools import wraps

import time
import numpy as np


def timer(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        print(f"{func.__name__} executed in {end_time - start_time:.4f} seconds")
        return result

    return wrapper


def inverse_rigid_transform(arr: np.ndarray) -> np.ndarray:
    Rt = arr[:3, :3].T
    tt = -np.dot(Rt, arr[:3, 3])
    return np.vstack((np.column_stack((Rt, tt)), [0, 0, 0, 1]))


def crop_pointcloud(pointcloud, lim_x, lim_y, lim_z):
    """
    Crop the pointcloud to the specified limits.
    """
    mask = (
        (pointcloud[:, 0] >= lim_x[0])
        & (pointcloud[:, 0] <= lim_x[1])
        & (pointcloud[:, 1] >= lim_y[0])
        & (pointcloud[:, 1] <= lim_y[1])
        & (pointcloud[:, 2] >= lim_z[0])
        & (pointcloud[:, 2] <= lim_z[1])
    )
    return pointcloud[mask]
