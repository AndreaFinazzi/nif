import math
import numpy as np

# Constants
r2d = 180/math.pi
d2r = math.pi/180
m2ft = 3.28084
ft2m = 0.3048
km2ft = m2ft * 1000

# Util functions
def clip(min_val, max_val, val):
    return min(max_val, max(min_val, val))

def euler2dcm(gamma, theta, psi):
    """
    Calculate direction cosine matrix from Euler angels
    Parameters
    ----------
    gamma: float, rad, roll
    theta: float, rad, pitch
    psi: float, rad, yaw
    Returns
    -------
    DCMbn: 3x3 numpy array DCM from body to navigation ref system
    References
    ----------
    [1] Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION TECHNOLOGY
    """

    # rotation psi about z axis
    C1 = np.array([
        [cos(psi),  sin(psi),   0],
        [-sin(psi), cos(psi),   0],
        [0,         0 ,         1]])

    # rotation theta about y axis
    C2 = np.array([
        [cos(theta),    0,  -sin(theta)] ,
        [0,             1,  0],
        [sin(theta),    0,  cos(theta)]])

    # rotation gamma about x axis
    C3 = np.array([
        [1,     0,          0],
        [0,     cos(gamma), sin(gamma)],
        [0,     -sin(gamma),    cos(gamma)]])

    # calculate body to navigation DCM
    dcm_bn = np.dot(np.dot(C1.T , C2.T) , C3.T)

    return dcm_bn

def eular2dcm(r, p, y):
    # c1 = math.cos(d2r*r)
    # c2 = math.cos(d2r*p)
    # c3 = math.cos(d2r*y)

    # s1 = math.sin(d2r*r)
    # s2 = math.sin(d2r*p)
    # s3 = math.sin(d2r*y)
    c1 = math.cos(r)
    c2 = math.cos(p)
    c3 = math.cos(y)

    s1 = math.sin(r)
    s2 = math.sin(p)
    s3 = math.sin(y)

    # yaw
    yaw_matrix = [
        [c3, 0, s3],
        [0, 1, 0],
        [-s3, 0, c3]
    ]

    # pitch
    pitch_matrix = [
        [1, 0,  0],
        [0, c2, s2],
        [0, -s2, c2]
    ]

    # roll
    roll_matrix = [
        [c1, -s1, 0],
        [s1, c1, 0],
        [0, 0, 1]
    ]

    temp1 = np.matmul(np.array(roll_matrix), np.array(pitch_matrix))
    temp2 = np.matmul(temp1, np.array(yaw_matrix))
    return temp2

def get_norm(x, y, z):
    mag = math.sqrt(x**2 + y**2 + z**2)
    if(mag == 0):
        return [0, 0, 0]
    return [x/mag, y/mag, z/mag]

def get_norm_2d(x, y):
    mag = math.sqrt(x**2 + y**2)
    if(mag == 0):
        return [0, 0, 0]
    return [x/mag, y/mag]

def make_deg_bun_sec(x):
    deg = int(x)
    bun = int((x - deg) * 60)
    sec = int((((x - deg) * 60) - bun) * 1000000 * 60 / 10000) / 100.0
    return [deg, bun, sec]

def normalize_pi2pi(angle):
    if angle > np.pi:
        return angle - 2*np.pi
    elif angle < -np.pi:
        return angle + 2*np.pi
    else:
        return angle

def AC2Robotics_coordinate(x_AC, y_AC, z_AC, roll_AC, pitch_AC, yaw_AC):
    """
    Tranform from Simulator to Robotics coordinate in global frame

    roll_AC  : [-0.5*pi, +0.5*pi]
    pitch_AC : [-0.5*pi, +0.5*pi]
    yaw_AC   : [      0, +2.0*pi]
    ---
    roll_robotics  : [-0.5*pi, +0.5*pi]
    pitch_robotics : [-0.5*pi, +0.5*pi]
    yaw_robotics   : [-0.5*pi, +0.5*pi]

    <Position>
    x_robotics = z_AC
    y_robotics = x_AC
    z_robotics = y_AC

    <Orientation>
    roll_robotics  = roll_AC
    pitch_robotics = - pitch_AC
    yaw_robotics   = -(normalize_pi2pi(yaw_AC) - 0.5*pi)
    # Simulator -> normalize to pi2pi -> - 90deg bias -> +- conversion -> Robotics
    """

    # x_AC, y_AC, z_AC, roll_AC, pitch_AC, yaw_AC = pose_AC

    x_robotics = -1 * z_AC
    y_robotics = -1 * x_AC
    z_robotics = -1 * y_AC
    roll_robotics = roll_AC
    pitch_robotics = pitch_AC
    yaw_robotics = yaw_AC

    pose_robotics = np.array([x_robotics, y_robotics, z_robotics, roll_robotics, pitch_robotics, yaw_robotics])

    return pose_robotics

def rpy2Rmatrix(roll, pitch, yaw):
    """
    Compute R matrix w.r.t. roll, pitch, yaw in Robotics coordinate
    """
    c_roll, c_pitch, c_yaw = math.cos(roll), math.cos(pitch), math.cos(yaw)
    s_roll, s_pitch, s_yaw = math.sin(roll), math.sin(pitch), math.sin(yaw)

    # roll (R_phi)
    roll_matrix = np.array(
    [
        [1,      0,       0, 0],
        [0, c_roll, -s_roll, 0],
        [0, s_roll,  c_roll, 0],
        [0,      0,       0, 1]
    ])

    # pitch (R_theta)
    pitch_matrix = np.array(
    [
        [ c_pitch, 0, s_pitch, 0],
        [       0, 1,       0, 0],
        [-s_pitch, 0, c_pitch, 0],
        [       0, 0,       0, 1]
    ])

    # yaw (R_psi)
    yaw_matrix = np.array(
    [
        [c_yaw, -s_yaw, 0, 0],
        [s_yaw,  c_yaw, 0, 0],
        [    0,      0, 1, 0],
        [    0,      0, 0, 1]
    ])
    
    # Return R_phi * R_theta * R_psi
    return np.matmul(np.matmul(np.array(roll_matrix), np.array(pitch_matrix)), np.array(yaw_matrix))

def xyz2Tmatrix(x, y, z):
    """
    Compute T matrix w.r.t. x, y, z in Robotics coordinate
    """
    T_matrix = np.array(
    [
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

    return T_matrix

def global_to_local_coordinate(global_pose, local_coordinate_pose):
    """
    target_pose: [x, y, z, roll, pitch, yaw]
    local_coordinate_pose: [x, y, z, roll, pitch, yaw]
    """
    local_pose = []
    x, y, z, roll, pitch, yaw = global_pose
    x0, y0, z0, roll0, pitch0, yaw0 = local_coordinate_pose

    R_matrix = rpy2Rmatrix(-roll0, -pitch0, -yaw0)
    T_matrix = xyz2Tmatrix(-x0, -y0, -z0)

    global_xyz1 = np.array(
    [
        [x],
        [y],
        [z],
        [1]
    ])

    local_xyz1 = np.matmul(np.matmul(R_matrix, T_matrix), global_xyz1)
    local_xyz = (local_xyz1[:-1].T)[0] # (4,1) to (3,1) to (3,)
    local_rpy = np.array([roll - roll0, pitch - pitch0, yaw - yaw0])

    local_pose = np.hstack([local_xyz, local_rpy]) # concat [x,y,z] and [roll,pitch,yaw]

    return local_pose