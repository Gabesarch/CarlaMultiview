import numpy as np 
import os
import time
import math
# import torch

def get_sensor_name(sensor):
    return sensor.split('.')[-1]

def get_extrinsics_for_data(data):
    loc = data.transform.location
    rot = data.transform.rotation
    extr = np.array([loc.x, loc.y, loc.z, rot.pitch, rot.yaw, rot.roll])
    return extr

def get_extrinsics_for_vehicle(data):
    loc = data.location
    rot = data.rotation
    extr = np.array([loc.x, loc.y, loc.z, rot.pitch, rot.yaw, rot.roll])
    return extr

# pitch, yaw, roll, x, y, z, scalex, scaley, scalez
def create_transformation_matrix(rotation, position, scale):
    # Transformation matrix
    pitch, yaw, roll = rotation
    tx, ty, tz = position
    scalex, scaley, scalez = scale
    cy = math.cos(np.radians(yaw))
    sy = math.sin(np.radians(yaw))
    cr = math.cos(np.radians(roll))
    sr = math.sin(np.radians(roll))
    cp = math.cos(np.radians(pitch))
    sp = math.sin(np.radians(pitch))
    matrix = np.eye(4)
    matrix[0, 3] = tx
    matrix[1, 3] = ty
    matrix[2, 3] = tz
    matrix[0, 0] = scalex * (cp * cy)
    matrix[0, 1] = scaley * (cy * sp * sr - sy * cr)
    matrix[0, 2] = -scalez * (cy * sp * cr + sy * sr)
    matrix[1, 0] = scalex * (sy * cp)
    matrix[1, 1] = scaley * (sy * sp * sr + cy * cr)
    matrix[1, 2] = scalez * (cy * sr - sy * sp * cr)
    matrix[2, 0] = scalex * (sp)
    matrix[2, 1] = -scaley * (cp * sr)
    matrix[2, 2] = scalez * (cp * cr)
    return matrix

def safe_inverse(a): #parallel version
    
    inv = np.copy(a)
    r_transpose = a[:3, :3].T #inverse of rotation matrix

    inv[:3, :3] = r_transpose
    inv[:3, 3:4] = -np.matmul(r_transpose, a[:3, 3:4])

    return inv

# def apply_4x4(RT, xyz):
#     B, N, _ = list(xyz.shape)
#     ones = torch.ones_like(xyz[:,:,0:1])
#     xyz1 = torch.cat([xyz, ones], 2)
#     xyz1_t = torch.transpose(xyz1, 1, 2)
#     # this is B x 4 x N
#     xyz2_t = torch.matmul(RT, xyz1_t)
#     xyz2 = torch.transpose(xyz2_t, 1, 2)
#     xyz2 = xyz2[:,:,:3]
#     return xyz2