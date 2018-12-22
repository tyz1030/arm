import numpy as np
#expm is a matrix exponential function
from scipy.linalg import expm
import copy

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""
def get_A_matrix(a, d, alpha, theta):
    ''' Returns A matrix transforming one, relying on dv convention '''
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_alpha = np.cos(alpha)
    s_alpha = np.sin(alpha)

    return np.array([
                    [c_theta, -s_theta * c_alpha, s_theta * s_alpha, a * c_theta],
                    [s_theta, c_theta * c_alpha, -c_theta * s_alpha, a * s_theta],
                    [0.0, s_alpha, c_alpha, d],
                    [0.0, 0.0, 0.0, 1.0]
                    ])

def FK_dh(rexarm):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

 
    return a 4-tuple (x, y, z, phi) representing the pose of end effector

    note: phi is the euler angle about the y-axis in the base frame

    """
    # prepare dv variables for 5 A matrices
    [l1, l2, l3, l4, l5] = rexarm.link_lengths
    a = [0.0, l2, l2, 0.0, 0.0]
    d = [l1, 0.0, 0.0, 0.0, l4 + l5]
    alpha = [np.pi/2.0, 0.0, 0.0, np.pi/2.0, 0.0] 
    theta = rexarm.joint_angles_fb
    theta += np.array([np.pi/2.0, np.pi/2.0, 0.0, np.pi/2.0, 0.0])
   
    # Get transformation matrix 
    A = np.zeros((4,4,5))
    T = np.identity(4)
    for i in range(0,5):
        T = np.matmul(T, get_A_matrix(a[i], d[i], alpha[i], theta[i]))

    return get_pose_from_T(T)
    
def FK_pox(rexarm, end_pose_only=True, theta=None):
    """
    TODO: implement this function

    Calculate forward kinematics for rexar
    using product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of end effector

    note: phi is the euler angle about y in the base frame

    """

    [l1, l2, l3, l4, l5] = rexarm.link_lengths

    if theta is None:
        theta = copy.copy(rexarm.joint_angles_fb)
    theta[4] += 35.0 * np.pi / 180.0

    M1 = np.array([[0.0, 0.0, 1.0, 0.0],
                   [-1.0, 0.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0, l1],
                   [0.0, 0.0, 0.0, 1.0]
                   ])

    M2 = np.array([[0.0, 0.0, 1.0, 0.0],
                   [-1.0, 0.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0, l1+l2],
                   [0.0, 0.0, 0.0, 1.0]
                   ])
    
    M3 = np.array([[0.0, 0.0, 1.0, 0.0],
                   [-1.0, 0.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0, l1+l2+l3],
                   [0.0, 0.0, 0.0, 1.0]
                   ])

    M4 = np.array([[0.0, 1.0, 0.0, 0.0],
                   [-1.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0, l1+l2+l3],
                   [0.0, 0.0, 0.0, 1.0]
                   ])
    
    M5 = np.array([[0.0, 1.0, 0.0, 0.0],
                   [-1.0, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0, l1+l2+l3+l4+l5],
                   [0.0, 0.0, 0.0, 1.0]
                   ])
    
    S1 = np.asarray([[0.0, -1.0, 0.0, 0.0],
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0]
                    ])
    S2 = np.asarray([[0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0, l1],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0]
                    ])
    S3 = np.asarray([[0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0, l1+l2],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0]
                    ])
    S4 = np.asarray([[0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, -1.0, l1+l2+l3],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0]
                    ])
    
    S5 = np.asarray([[0.0, 1.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0]
                    ])
    
    tmp2 = np.matmul(expm(S1*theta[0]), expm(S2*theta[1]))
    tmp3 = np.matmul(tmp2, expm(S3*theta[2]))
    tmp4 = np.matmul(tmp3, expm(S4*theta[3]))
    tmp5 = np.matmul(tmp4, expm(S5*theta[4]))

    T2 = np.matmul(tmp2, M2)
    T3 = np.matmul(tmp3, M3)
    T4 = np.matmul(tmp4, M4)
    T5 = np.matmul(tmp5, M5)

    # For display, only need end pose
    # For IK, need to each joint position to check constraints
    if end_pose_only:
        return get_pose_from_T(T5) 
    else:
        return get_pose_from_T(T5), get_pose_from_T(T4), get_pose_from_T(T3), get_pose_from_T(T2)
    
def IK(rexarm, end_pose, omega, phi=0.0, offset=True, ori=0.0, verbose=False):
    """
    Calculate inverse kinematics for rexarm

    return the required joint angles
    omega is angle of last arm link to xy plane
    phi is angle of wrist rotation about end effector z
    orientation is orientation of block (if necessary)

    """
    pos = end_pose.astype(np.float32)
    theta = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    [a1, a2, a3, a4, a5] = rexarm.link_lengths
    a4 += a5
    r2d = 180.0 / np.pi

    if verbose:
        print('Initial pos', pos)

    theta[0] = find_base_angle(pos)

    # Offset attempts to deal with mismatch between z axis of gripper and axis of link 
    if offset:
        if omega < -70*np.pi/180:
            pos[2] -= 0.01
        
        d_from_origin = (pos[0]**2 + pos[1]**2 + (pos[2]-a1)**2)**0.5
        base_offset = np.arctan2(0.008, d_from_origin)
        
        if verbose:
            print('BASE_OFFSET', base_offset)
        
        if omega >= -30*np.pi/180.0:
            unit_normal = np.array([0.0, 0.0, 1.0])
        else:
            unit_normal = np.array([np.sin(ori), np.cos(ori), 0])
        
        #pos += 0.02*unit_normal
        
        theta[0] = find_base_angle(pos)
        theta[0] -= base_offset

    if verbose:
        print('BASE ANGLE', r2d*theta[0])
        print('PHI', r2d*phi)
        print('OMEGA', r2d*omega)
        print('Pos after offset', pos)

    theta[4] = phi

    # Now we move into the plane created by the arm once base has moved, i.e
    p3 = np.array([np.sqrt(pos[0]**2 + pos[1]**2), pos[2] - a1])
    if verbose: 
        print('END EFFECTOR IN PLANE', p3)
    
    # get the position of the wrist in our plane
    wrist = find_wrist(p3, omega, a4)
    if verbose:
        print('WRIST LOC IN PLANE', wrist)
    
    # With the wrist point, find shoulder and elbow angles
    theta[1], theta[2] = IK_2R_planar(wrist, a2, a3, elbow_up=True)

    # get the wrist angle 
    theta[3], p1 = find_wrist_angle(theta, a2, omega)
    theta[1] -= np.pi/2
    if verbose:
        print('SHOULDER', r2d*theta[1])
        print('ELBOW', r2d*theta[2])
        print('WRIST', r2d*theta[3])

    # wrap around angles
    if theta[4] > 120*np.pi/180:
        theta[4] -= np.pi
    elif theta[4] < -120*np.pi/180:
        theta[4] += np.pi
    
    if theta[0] > 180*np.pi/180:
        theta[0] -= 2*np.pi
    elif theta[0] < -180*np.pi/180:
        theta[0] += 2*np.pi

    # check solution is valid
    if not check_valid(theta, p1, wrist, a1):
        # check for another solution
        if verbose:
            print('invalid solution, checking another...')
        theta[1], theta[2] = IK_2R_planar(wrist, a2, a3, elbow_up=False)
        theta[3], p1 = find_wrist_angle(theta, a2, omega)
        if not check_valid(theta, p1, wrist, a1):
            raise Exception('no valid solution')
        
    # get forward kinematics to check
    #p3f, p2f, p1f = FK_pox(rexarm, end_pose_only=False, theta=theta)
    
    return theta


def find_base_angle(pos):
    if pos[0] == 0:
        if pos[1] > 0:
            return 0.0
        return -np.pi
    return np.arctan2(pos[0], pos[1])

def find_wrist(p3, phi, L3):
    p2 = p3 - L3* np.array([np.cos(phi), np.sin(phi)])
    return p2

def IK_2R_planar(p2, L1, L2, elbow_up=True):
    D = np.sum(p2**2)
    D = np.round(D, 6)

    #print('D', D)
    tmp = (L1**2 + L2**2 - D) / (2*L1*L2)
    #print('tmp', tmp)
    beta = np.arccos(tmp)

    #print('beta', beta)
    theta2 = beta - np.pi
    
    gamma = np.arctan2(p2[1],p2[0])
    tmp = (L1**2 + D - L2**2) / (2*L1*np.sqrt(D))
    alpha = np.arccos(tmp)
    theta1 = gamma + alpha
    
    if not elbow_up:
        theta1 = gamma - alpha
        theta2 *= -1
        
    return theta1, theta2

def find_wrist_angle(theta, L1, phi):
    [x1, y1] = L1 * np.array([np.cos(theta[1]), np.sin(theta[1])])

    #D = (p3[0] - x1)**2 + (p3[1]-y1)**2
    #tmp = (L3**2 + L2**2 - D) / (2*L3*L2)
    #beta = np.arccos(tmp)
    #heta3 = -(beta - np.pi) 
    theta3 = phi - theta[2] -theta[1]
    return theta3, np.array([x1, y1])

def check_valid(theta, p1, p2, a1):
    max_vals = np.array([180, 120, 120, 120, 120]) * np.pi/180
    min_vals = -max_vals
    
    if np.isnan(theta).any():
        #print('nan in angles')
        return False
        
    if p1[1] < -a1 or p2[1] < -a1:
        #print('would hit floor')
        return False
        
    if (theta > max_vals).any() or (theta < min_vals).any():
        #print('theta out of range')
        #print(theta)
        return False
    
    return True


def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """

    R = T[:3, :3]

    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0.0

    return x, y, z

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """

    [x, y, z, _] = np.matmul(T, np.array([0.0, 0.0, 0.0, 1.0]))
    _, phi, _ = get_euler_angles_from_T(T)
    phi *= 180.0/np.pi
    return (x, y, z, phi)
