#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import pi
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.array([[0, -1, 0, 0.392],
              [0, 0, -1, 0.432],
              [1, 0, 0, 0.2155],
              [0, 0, 0, 1]])
	S = np.zeros((6,6))
	w1 = np.array([0, 0, 1])
	q1 = np.array([-0.15, 0.15, 0.01])
	v1 = -np.cross(w1, q1)
	S[0:3, 0] = w1
	S[3:6, 0] = v1

	w2 = np.array([0, 1, 0])
	q2 = np.array([-0.15, 0.27, 0.162])
	v2 = -np.cross(w2, q2)
	S[0:3, 1] = w2
	S[3:6, 1] = v2

	w3 = np.array([0, 1, 0])
	q3 = np.array([0.094, 0.27, 0.162])
	v3 = -np.cross(w3, q3)
	S[0:3, 2] = w3
	S[3:6, 2] = v3

	w4 = np.array([0, 1,0])
	q4 = np.array([0.307, 0.177, 0.162])
	v4 = -np.cross(w4, q4)
	S[0:3, 3] = w4
	S[3:6, 3] = v4

	w5 = np.array([1,0,0])
	q5 = np.array([0.307, 0.281, 0.162])
	v5 = -np.cross(w5, q5)
	S[0:3, 4] = w5
	S[3:6, 4] = v5

	w6 = np.array([0,1,0])
	q6 = np.array([0.392, 0.281, 0.162])
	v6 = -np.cross(w6, q6)
	S[0:3, 5] = w6
	S[3:6, 5] = v6

	# ==============================================================#
	return M, S

def VecTose3(V):
    """Converts a spatial velocity vector into a 4x4 matrix in se3
    :param V: A 6-vector representing a spatial velocity
    :return: The 4x4 se3 representation of V
    Example Input:
        V = np.array([1, 2, 3, 4, 5, 6])
    Output:
        np.array([[ 0, -3,  2, 4],
                  [ 3,  0, -1, 5],
                  [-2,  1,  0, 6],
                  [ 0,  0,  0, 0]])
    """
    return np.r_[np.c_[VecToso3([V[0], V[1], V[2]]), [V[3], V[4], V[5]]],
                 np.zeros((1, 4))]

def VecToso3(omg):
    """Converts a 3-vector to an so(3) representation
    :param omg: A 3-vector
    :return: The skew symmetric representation of omg
    Example Input:
        omg = np.array([1, 2, 3])
    Output:
        np.array([[ 0, -3,  2],
                  [ 3,  0, -1],
                  [-2,  1,  0]])
    """
    return np.array([[0,      -omg[2],  omg[1]],
                     [omg[2],       0, -omg[0]],
                     [-omg[1], omg[0],       0]])

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#

	T = Get_MS()[0]
	T = expm(VecTose3(Get_MS()[1][:, 0]) * 
	theta1)@expm(VecTose3(Get_MS()[1][:, 1]) * 
	theta2)@expm(VecTose3(Get_MS()[1][:, 2]) * 
	theta3)@expm(VecTose3(Get_MS()[1][:, 3]) * 
	theta4)@expm(VecTose3(Get_MS()[1][:, 4]) * 
	theta5)@expm(VecTose3(Get_MS()[1][:, 5]) * 
	theta6)@T
	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0
	
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
