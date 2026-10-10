import numpy as np


FRICTION_VELOCITY_SCALE = 0.1


def smooth_sign(velocity, velocity_scale=FRICTION_VELOCITY_SCALE):
    return np.tanh(np.asarray(velocity) / velocity_scale)
