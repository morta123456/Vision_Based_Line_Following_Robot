import numpy as np
from math import cos, sin

def rotate_vector(vec, angle_deg, anchor=None):
    theta = np.deg2rad(angle_deg)
    rot = np.array([
        [cos(theta), -sin(theta)],
        [sin(theta), cos(theta)]]    
    )
    if anchor is None:
        anchor = (0,0)
    return np.dot(rot, vec - anchor) + anchor

def lerp(A, B, ratio):
    return (A * ratio) + ((1 - ratio) * B)