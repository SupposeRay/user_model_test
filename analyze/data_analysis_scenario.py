#!/usr/bin/python3

import numpy as np
import os, sys
import readline
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial.distance import euclidean, directed_hausdorff
from dtw import dtw # https://dynamictimewarping.github.io/python/ pip3/pip install dtw-python

readline.set_completer_delims(' \t\n=')
readline.parse_and_bind('tab: complete')

# load file
if(len(sys.argv) < 2):
    file_directory = input('Please enter the full path directory to log file including extension: ')
else:
    file_directory = sys.argv[1]

file_directory = os.path.expanduser(file_directory)

# open file, exit if failed
try:
    file_data = open(str(file_directory), "r")
except:
    print("Unable to open file, please check if the file exists or it is permitted to read.")
    exit()

# Read until end and convert each value into numbers
# Paths are stored as np.arrays of arrays, each coordinate is stored in an array of 2 elements
ref_path = np.empty((0,2), float)
user_path = np.empty((0,2), float)
user_time = np.array([])
user_input = np.empty((0,2), float)
robot_velocity = np.empty((0,2), float)

# Read away the header line
file_data.readline()

# ref.x,ref.y,ref.z,ref.o.x,ref.o.y,ref.o.z,ref.o.w,user.time,user.x,user.y,user.z,user.o.x,user.o.y,user.o.z,user.o.w,user.linvel,user.angvel,robot.linvel,robot.angvel
while 1:
    line = file_data.readline()
    if len(line) == 0:
        break
    
    # Split line without including newline character \n
    split_string = line[0: len(line) - 1].split(",")

    # Add to ref_path
    new_ref_pose = np.array([[float(split_string[0]), float(split_string[1])]])
    if not (math.isnan(new_ref_pose[0][0]) and math.isnan(new_ref_pose[0][1])):
        ref_path = np.append(ref_path, new_ref_pose, axis=0)

    # Add to user_path
    new_user_pose = np.array([[float(split_string[8]), float(split_string[9])]])
    if not (math.isnan(new_user_pose[0][0]) and math.isnan(new_user_pose[0][1])):
        user_path = np.append(user_path, new_user_pose, axis=0)

    # Add to user_input
    new_user_input = np.array([[float(split_string[19]), float(split_string[20])]])
    if not (math.isnan(new_user_input[0][0]) and math.isnan(new_user_input[0][1])):
        user_input = np.append(user_input, new_user_input, axis=0)

    # Add to user_time
    if not (math.isnan(float(split_string[7]))):
        user_time = np.append(user_time, float(split_string[7]))

    # Add to robot_velocity
    new_robot_vel = np.array([[float(split_string[17]), float (split_string[18])]])
    if not (math.isnan(new_robot_vel[0][0]) and math.isnan(new_robot_vel[0][1])):
        robot_velocity = np.append(robot_velocity, new_robot_vel, axis=0)

# Plot user path and reference path
user_path_x, user_path_y = zip(*user_path)
ref_path_x, ref_path_y = zip(*ref_path)
file_data.close()

def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

# Compute deviation from reference path using different metrics
try:
    path_dtw = dtw(user_path, ref_path)
except:
    print("Normalized dtw was unable to be calculated")
# hausdorff_dist = directed_hausdorff(ref_path, user_path)[0]

user_index = path_dtw.index1
ref_index = path_dtw.index2
print(len(ref_path_x))
print(len(user_path_x))
print(user_index[1400:1550])
print(ref_index[1400:1550])
# print(ref_index)
# print(len(user_index))
# print(len(ref_index))

dtw_length = len(path_dtw.index1)

# vector_user = np.empty((0,2), float)
# vector_ref = np.empty((0,2), float)
angle_all = np.array([])

for i in range(dtw_length):
    if i == 0:
        continue
    if i == 1459:
        print(user_index[i])
        break
    vector_ref = np.array([ ref_path_x[ref_index[i]]-ref_path_x[ref_index[i-1]], ref_path_y[ref_index[i]]-ref_path_y[ref_index[i-1]] ])
    vector_user = np.array([ user_path_x[user_index[i]]-user_path_x[user_index[i-1]], user_path_y[user_index[i]]-ref_path_y[user_index[i-1]] ])
    if np.linalg.norm(vector_ref) == 0 or np.linalg.norm(vector_user) == 0:
        continue
    else:
        angle_diff = angle_between(vector_ref, vector_user)
        if vector_ref[0] * vector_user[1] - vector_ref[1] * vector_user[0] < 0:
            angle_diff = - angle_diff
        angle_all = np.append(angle_all, angle_diff)


# dtw_length = len(path_dtw.index1)

# vector_user = np.empty((0,2), float)
# vector_ref = np.empty((0,2), float)
# angle_all = np.array([])

# for i in range(dtw_length):
#     if i == 0:
#         continue
#     if user_index[i] == user_index[i-1] and ref_index[i] != ref_index[i-1]:
#         vector_ref = np.append(vector_ref, np.array([[ ref_path_x[ref_index[i]]-ref_path_x[ref_index[i] - 1], ref_path_y[ref_index[i]]-ref_path_y[ref_index[i] - 1] ]]), axis=0)
#     elif ref_index[i] == ref_index[i-1] and user_index[i] != user_index[i-1]:
#         vector_user = np.append(vector_user, np.array([[ user_path_x[user_index[i]]-user_path_x[user_index[i] - 1], user_path_y[user_index[i]]-ref_path_y[user_index[i] - 1] ]]), axis=0)
#     else:
#         vector_ref = np.append(vector_ref, np.array([[ ref_path_x[ref_index[i]]-ref_path_x[ref_index[i] - 1], ref_path_y[ref_index[i]]-ref_path_y[ref_index[i] - 1] ]]), axis=0)
#         vector_user = np.append(vector_user, np.array([[ user_path_x[user_index[i]]-user_path_x[user_index[i] - 1], user_path_y[user_index[i]]-ref_path_y[user_index[i] - 1] ]]), axis=0)
#         if vector_ref.shape[0] <= vector_user.shape[0]:# compare previous user with the current ref
#             for j in range(vector_user.shape[0]):
#                 if np.linalg.norm(vector_ref[0]) == 0 or np.linalg.norm(vector_user[j]) == 0:
#                     continue
#                 else:
#                     angle_diff = angle_between(vector_ref[0], vector_user[j])
#                     if vector_ref[0][0] * vector_user[j][1] - vector_ref[0][1] * vector_user[j][0] < 0:
#                         angle_diff = - angle_diff
#                     angle_all = np.append(angle_all, angle_diff)
#         else:# compare previous ref with the current user
#             for j in range(vector_ref.shape[0]):
#                 if np.linalg.norm(vector_ref[j]) == 0 or np.linalg.norm(vector_user[0]) == 0:
#                     continue
#                 else:
#                     angle_diff = angle_between(vector_ref[j], vector_user[0])
#                     if vector_ref[j][0] * vector_user[0][1] - vector_ref[j][1] * vector_user[0][0] < 0:
#                         angle_diff = - angle_diff
#                     angle_all = np.append(angle_all, angle_diff)
#         vector_ref = np.delete(vector_ref,np.s_[:], 0)
#         vector_user = np.delete(vector_user,np.s_[:], 0)

plt.hist(angle_all)
plt.show()

# vector_user = np.array([[3,4],[1,2]])
# vector_ref = np.array([[5,6]])
# vector_user = np.append(vector_user, vector_ref, axis=0)
# print(vector_user)
# print(vector_user.shape[0])
# print(vector_user[0][0])
# print(vector_user[2][1])
# vector_user = np.delete(vector_user,np.s_[:], 0)
# print(vector_user.shape[0])
# vector_user = np.empty((0,2), float)
# vector_user = np.append(vector_user, vector_ref, axis=0)
# norm_user = np.linalg.norm(vector_user[0])
# print(vector_user[0]/norm_user)

# if(len(sys.argv) > 2):
#     plt.show()