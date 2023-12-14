#!/usr/bin/python3

import numpy as np
import os, sys
import readline
import math
import matplotlib.pyplot as plt

readline.set_completer_delims(' \t\n=')
readline.parse_and_bind('tab: complete')

# load file
if(len(sys.argv) < 2):
    file_directory = input('Please enter the full path directory to log file including extension: ')
else:
    file_directory = sys.argv[1]

file_directory = os.path.expanduser(file_directory)

if str(file_directory)[-9:] == "trace.txt":
    analysis_mode = 'trace'
elif str(file_directory)[-9:] == "point.txt":
    analysis_mode = 'point'
else:
    print("The file name is not accepted, please use file ended with \"point.txt\" or \"trace.txt\".")
    exit()

# open file, exit if failed
try:
    file_data = open(str(file_directory), "r")
except:
    print("Unable to open file, please check if the file exists or it is permitted to read.")
    exit()


# declare variables
# ref_path = np.empty((0,2), float)
# ref_point = np.empty((0,2), float)
# user_path = np.empty((0,3), float)
# user_time = np.array([])
# user_input = np.empty((0,2), float)

# read the first line to remove the header row
file_data.readline()

if (analysis_mode == 'point'):
    ref_point = np.empty((0,3), float)
    user_path = np.empty((0,3), float)
    trial_num = 0
    update_ref = False
    while True:
        data_line = file_data.readline()
        if len(data_line) == 0:
            break
        
        # split the line by comma ","
        split_string = data_line[0: len(data_line) - 1].split(",")

        # check trial index
        if (trial_num != int(split_string[0])):
            if (update_ref == False):
                update_ref = True
            else:
                trial_num = int(split_string[0])
                # add to reference
                ref_point = np.append(ref_point, np.array([[float(split_string[2]), float(split_string[3]), float(trial_num)]]), axis=0)
                update_ref = False
        # add to user_path
        if (trial_num !=0):
            user_path = np.append(user_path, np.array([[float(split_string[10]), float(split_string[11]), float(trial_num)]]), axis=0)
    
    # plot the data
    start_point_user = 0
    end_point_user = len(user_path)

    # plot_fig = plt.figure('pointing_test')
    # plt.plot(960, -540, label="Origin", color="lime", marker="o", markersize=10.0, ls="")
    # for i in range(int(ref_point[-1][-1])):
        
    #     for j in range(start_point_user, end_point_user):
    #         if user_path[j][2] == i + 1 and j != end_point_user - 1:
    #             continue
    #         else:
    #             if j == end_point_user - 1:
    #                 j = j + 1
    #             user_path_x, user_path_y, _ = zip(*user_path[start_point_user:j])
    #             start_point_user = j
    #             break
    #     # if user_path_x[0] - ref_point[i][0] - 
    #     plt.plot(user_path_x, user_path_y, label="User's path", marker=".")
    #     plt.plot(user_path_x[0], user_path_y[0], color="red", marker="o", markersize=5.0, ls="")
    #     plt.plot(ref_point[i][0], ref_point[i][1], color="darkorange", label="Reference point", marker="o", markersize=8.0, ls="")

    # plt.title("Pointing Test")
    # plt.xlabel("x-coordinate")
    # plt.ylabel("y-coordinate")
    # plt.xlim([550,1370])
    # plt.ylim([-130, -950])
    # # plt.legend()
    # plt.grid(True)
    # # plt.axis('square')

    for i in range(int(ref_point[-1][-1])):
        
        for j in range(start_point_user, end_point_user):
            if user_path[j][2] == i + 1 and j != end_point_user - 1:
                continue
            else:
                if j == end_point_user - 1:
                    j = j + 1
                user_path_x, user_path_y, _ = zip(*user_path[start_point_user:j])
                start_point_user = j
                break
        
        if (i % 4 == 0):
            plot_fig = plt.figure('pointing_test ' + str(math.floor(i/4) + 1))
        plt.subplot(2, 2, (i % 4) +1)
        plt.plot(960, -540, label="Origin", color="lime", marker="o", markersize=10.0, ls="")
        plt.plot(user_path_x, user_path_y, label="User's path", marker=".")
        plt.plot(user_path_x[0:100], user_path_y[0:100], color="purple")
        plt.plot(user_path_x[0], user_path_y[0], color="red", marker="o", markersize=5.0, ls="")
        plt.plot(ref_point[i][0], ref_point[i][1], color="darkorange", label="Reference point", marker="o", markersize=8.0, ls="")
        plt.title("Pointing Test " + str(i+1))
        plt.xlabel("x-coordinate")
        plt.ylabel("y-coordinate")
        plt.xlim([550,1370])
        plt.ylim([-130, -950])
        # plt.legend()
        plt.grid(True)
        # plt.axis('scaled')
        # plt.axis('square')

elif (analysis_mode == 'trace'):
    ref_point = np.empty((0,3), float)
    temp_ref_point = np.array([0, 0, 0])
    user_path = np.empty((0,3), float)
    trial_num = 0
    while True:
        data_line = file_data.readline()
        if len(data_line) == 0:
            break
        
        # split the line by comma ","
        split_string = data_line[0: len(data_line) - 1].split(",")

        # check trial index
        if (trial_num != float(split_string[0])):
            trial_num = float(split_string[0])
        
        # add to reference
        if split_string[20] != 'nan':
            if not np.array_equal(temp_ref_point, np.array([float(split_string[20]), float(split_string[21]), float(trial_num)])):
                ref_point = np.append(ref_point, np.array([[float(split_string[20]), float(split_string[21]), float(trial_num)]]), axis=0)
                temp_ref_point = np.array([float(split_string[20]), float(split_string[21]), float(trial_num)])
        
            # add to user_path
            user_path = np.append(user_path, np.array([[float(split_string[9]), float(split_string[10]),  float(trial_num)]]), axis=0)
    
    # plot the data
    # plot_fig = plt.figure('tracing_test')

    start_point_user = 0
    end_point_user = len(user_path)
    start_point_ref = 0
    end_point_ref = len(ref_point)
    
    for i in range(int(ref_point[-1][-1])):

        for j in range(start_point_user, end_point_user):
            if user_path[j][2] == i + 1 and j != end_point_user - 1:
                continue
            else:
                if j == end_point_user - 1:
                    j = j + 1
                user_path_x, user_path_y, _ = zip(*user_path[start_point_user:j])
                start_point_user = j
                break
        for k in range(start_point_ref, end_point_ref):
            if ref_point[k][2] == i + 1 and k != end_point_ref - 1:
                continue
            else:
                if k == end_point_ref - 1:
                    k = k + 1
                ref_point_x, ref_point_y, _ = zip(*ref_point[start_point_ref:k])
                start_point_ref = k
                break
        
        plot_fig = plt.figure('pointing_test ' + str(i + 1))
        # plt.subplot(2, 2, i+1)
        plt.plot(ref_point_x, ref_point_y, color="darkorange", label="Reference point", marker="o", markersize=30.0, ls="")
        plt.plot(user_path_x, user_path_y, label="User's path", marker="*")
        # plt.plot(user_path_x[0:50], user_path_y[0:50], color="purple")
        plt.title("Tracing Test")
        plt.xlabel("x-coordinate")
        plt.ylabel("y-coordinate")
        # plt.legend()
        plt.grid(True)
        # plt.axis([600, 1370, -900, -200], 'scaled')
        # plt.axis('scaled')
        plt.xlim(600, 1370)
        plt.ylim(-900, -200)
        plt.gca().set_aspect('equal', adjustable='box')


file_data.close()

plt.show()