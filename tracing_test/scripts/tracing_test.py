#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from math import sqrt
import time
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import Tkinter as tk
import tkMessageBox
import os, sys
from playsound import playsound
from threading import Thread
from threading import Timer

input_x = 0.0
input_y = 0.0
raw_x = 0.0
raw_y = 0.0
update = False


# Path trace parameters & vertices set up
r = 10      # radius of cursor
edge_length = 400       # size of shape
num_sides = 4       # sides of polygon
distance_threshold = 20     # distance acceptable at vertices
moving_distance = 500       # moving range of cursor 
default_relax_countdown = 5     #intertrial break


# set up variables
relax_timer = None
relax_countdown = default_relax_countdown
relax_countdown_flag = False
log_data_flag = True
closing_window_flag = False
square_vertices = []
fixed_vertices = []
trial_number = 1
file_dir = ""
file_name = ""


# Logging lists
time_start = 0
user_poses = []
reference_poses = []
reference_point = []
user_time = []
user_input = []
raw_user_input = []
trial = []
flag = []



# track number of sides left
def timerCallback(event):
    global update 
    if not len(square_vertices) == 0:
        update = True



# input from joystick
def joystickCallback(Point):
    global input_x, input_y, prev_time, raw_x, raw_y
    # Place some filtering to prevent drifting
    raw_x = Point.y
    raw_y = Point.x
    if (abs(Point.x) >= 0.04):
        input_y = (math.floor(raw_y * 10.0))/10.0
    else:
        input_y = 0
    if (abs(Point.y) >= 0.04):
        input_x = (math.floor(raw_x * 10.0))/10.0
    else:
        input_x = 0
    # print("input: ",input_x, input_y)
    # print("point: ",Point.x,Point.y)



def twistCallback(Twist):
    global input_x, input_y
    input_y = Twist.linear.x
    input_x = Twist.angular.z



# Data to be saved
def save_logs():
    global file_dir_entry, file_name_entry, square_vertices, label_file_dir, label_file_name, save_button, log_data_flag, relax_timer, closing_window_flag
    clear_traces()
    file_dir = file_dir_entry.get()
    file_name = file_name_entry.get()

    if len(file_dir) == 0 or len(file_name) == 0:
        print("Please input a full save directory (including last '/') and file name with extension")
        return

    print("Saving logs to " + file_dir + file_name + "_trace.txt")
    # trial,ref.x,ref.y,ref.z,ref.o.x,ref.o.y,ref.o.z,ref.o.w,user.time,user.x,user.y,user.z,user.o.x,user.o.y,user.o.z,user.o.w,user.linvel,user.angvel,robot.linvel,robot.angvel
    f = open(file_dir + file_name + "_trace.txt", "w")
    if f.closed:
        print("Could not open file to save logs")
        return

    f.writelines("trial,ref.x,ref.y,ref.z,ref.o.x,ref.o.y,ref.o.z,ref.o.w,user.time,user.x,user.y,user.z,user.o.x,user.o.y,\
                user.o.z,user.o.w,user.linvel,user.angvel,raw.linvel,raw.angvel,ref_pt.x, ref_pt.y\n")

    for i in range(len(user_poses)):
        save_string = "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                        trial[i] if len(trial) > i else "nan",
                        reference_poses[i][0] if len(reference_poses) > i else "nan",
                        -reference_poses[i][1] if len(reference_poses) > i else "nan",
                        "nan",
                        "nan",
                        "nan",
                        "nan",
                        "nan",
                        user_time[i] if len(user_time) > i else "nan",
                        user_poses[i][0] if len(user_poses) > i else "nan",
                        -user_poses[i][1] if len(user_poses) > i else "nan",
                        "nan",
                        "nan",
                        "nan",
                        "nan",
                        "nan",
                        user_input[i][0] if len(user_input) > i else "nan",
                        -user_input[i][1] if len(user_input) > i else "nan",
                        raw_user_input[i][0] if len(raw_user_input) > i else "nan",
                        -raw_user_input[i][1] if len(raw_user_input) > i else "nan",
                        reference_point[i][0] if len(reference_point) > i else "nan",
                        -reference_point[i][1] if len(reference_point) > i else "nan")
        
        f.writelines(save_string)


    f.close()
    print("Successfully saved to logs")
    clear_traces()


    # after done saving, confirm if user wants to continue or quit
    if tkMessageBox.askokcancel('Saved', 'Do you want to start a new trial?'):
        os.execv(sys.argv[0], sys.argv)

    else:
        if tkMessageBox.askokcancel('Exit', 'Do you want to quit?'):
            # kill window if quit
            window.destroy()
            rospy.signal_shutdown('Shutting down ROS.')
        
        else:
            # continue logging data
            file_dir_entry.destroy()
            file_name_entry.destroy()
            label_file_dir.place_forget()
            label_file_name.place_forget()
            save_button.destroy()
            log_data_flag = True
            closing_window_flag = False
            # generate square if end of the trial
            if (len(square_vertices) - 1 ) == 0:
                if relax_timer is None:
                    # print("hello")
                    relax_timer = Timer(1.0, relax_tick)
                    relax_timer.start()


def clear_traces():
    canvas.delete("trace")



# clear log list if trials are to be recorded after one another
def clear_log_lists():
    global user_poses, reference_poses, user_time, user_input, raw_user_input
    user_poses = []
    reference_poses = []
    user_time = []
    user_input = []
    raw_user_input = []



# set up GUI 
def generate_window():
    global window, target_cursor, canvas, edge_length
    window = tk.Tk()
    window.attributes('-fullscreen', True)
    # window.state('iconic')
    window.overrideredirect(False)       # do not show tool bar
    window.bind('<Escape>', lambda event: closing_window())
    window.title('Tracing Test')
    window.update_idletasks()
    if window.winfo_screenwidth > window.winfo_width():
        window.geometry('+'+str(window.winfo_width())+'+0')
        window.update_idletasks()
    # interface appearance
    # window_interface = str(w) + 'x' + str(h)
    # window.geometry(window_interface)
    w, h = window.winfo_width(), window.winfo_height()    # width and height of window
    center_x, center_y = (w/2),(h/2)    # set up middle of interface
    window.geometry("{0}x{1}".format(w,h))   
    canvas = tk.Canvas(window, bg='black', highlightthickness=3, highlightbackground="black", height=h, width=w)

    # Generate 10px diameter circle
    target_cursor = canvas.create_oval( (center_x - (edge_length / 2) + r) , (center_y - (edge_length / 2) + r) , (center_x - (edge_length / 2) - r) , (center_y - (edge_length / 2) - r) , fill='white', tag='cursor')
    # Place canvas at top left position
    canvas.place(x=0, y=0, anchor='nw')
    return w, h, center_x, center_y



# set up entry to get the file name and location for the logged data to be saved at
def generate_file_path():
    global file_dir_entry, file_name_entry, file_dir, file_name, label_file_dir, label_file_name, save_button
    # entry for directory
    label_file_dir = tk.Label(window, text="File Directory", fg='black')
    label_file_dir.place(x=0, y=0)
    file_dir_entry = tk.Entry(window, fg='black')
    file_dir_entry.place(x = 0, y = 20)
    file_dir_entry.insert(1, str(file_dir))

    # entry for file name
    label_file_name = tk.Label(window, text="File Name", fg='black')
    label_file_name.place(x=0, y=80)
    file_name_entry = tk.Entry(window, fg='black')
    file_name_entry.place(x = 0, y = 100)
    file_name_entry.insert(1, str(file_name))

    # click "save" button to start generating file by calling the function save_logs
    save_button = tk.Button(window, text='Save',font=('Times', 20), fg='black', command=save_logs)
    save_button.place(x=0, y=160)



def generate_trace(update = False):
    global target_cursor, square_vertices, edge_length
    cursor_position = canvas.coords(target_cursor)
    # If not already drawn, draw polygon
    if not update:
        global time_start
        time_start = time.time() 
        # clear_log_lists()
        # square_vertices = []
        square_vertices.append(((cursor_position[0] + cursor_position[2])/2, (cursor_position[1] + cursor_position[3])/ 2))
        angle_interval = 2 * math.pi / num_sides        # each ext. angle
        if trial_number % 2 == 0:
            operation_x = 'sin'
            operation_y = 'cos'
        else:
            operation_x = 'cos'
            operation_y = 'sin'
        for side in range(num_sides):
            new_x = eval('edge_length * math.' + operation_x + '(angle_interval * side)')
            new_y = eval('edge_length * math.' + operation_y + '(angle_interval * side)')
            square_vertices.append((square_vertices[-1][0] + new_x, square_vertices[-1][1] + new_y))
            if trial_number == 1:
                if len(square_vertices) == 4:
                    fixed_vertices.append(square_vertices[0:4])
            else:
                if len(square_vertices) == 5:
                    fixed_vertices.append(square_vertices[1:])


        # print(len(square_vertices), square_vertices)
            # print(fixed_vertices)
    # Update first vertex to start from cursor position
    else:
        if not len(square_vertices) == 0:
            square_vertices[0] = ((cursor_position[0] + cursor_position[2])/2, (cursor_position[1] + cursor_position[3])/ 2)

    # Redraw trace
    clear_traces()
    for i in range(len(square_vertices) - 1):       # started square_vertices with original position, thus -1 for number of sides
        x1 = square_vertices[i][0]
        x2 = square_vertices[i + 1][0]
        y1 = square_vertices[i][1]
        y2 = square_vertices[i + 1][1]
        canvas.create_line(x1, y1, x2, y2, fill= ("red" if i == 0 else "yellow"), width=5, tag="trace")



def move_cursor(moving_distance):
    "set-up movement of cursors, moving_distance: maximum displacement from the starting point of moving cursor "
    global input_x, input_y, target_cursor

    # Set scaled coordinate increment based on user's input
    x_increment = -10 * input_x
    y_increment = -10 * input_y

    # Get current position of the cursor
    target_position = canvas.coords(target_cursor)
    
    # Check if cursor is at boundary, if yes then stop movement
    if (target_position[0] <= 0) and (x_increment < 0):
        x_increment = 0
    if (target_position[2] >= w) and (x_increment > 0):
        x_increment = 0
    if (target_position[1] <= 0) and (y_increment < 0):
        y_increment = 0
    if (target_position[3] >= h) and (y_increment > 0):
        y_increment = 0

    canvas.move(target_cursor, x_increment, y_increment)



# subject to rest after each successful trial
def relax_tick():
    global relax_countdown, relax_timer, relax_countdown_flag, trial_number, closing_window_flag
    relax_countdown = relax_countdown - 1 
    # print('countdown')
    if relax_countdown:
            relax_timer = Timer(1.0, relax_tick)
            relax_timer.start()
            
    else: # done resting
        if not closing_window_flag:
            relax_countdown_flag = True
            audio("go.mp3")
            # trial_number = trial_number + 1
            relax_timer = None
            relax_countdown = default_relax_countdown
    


# [x] window
def closing_window():
    global log_data_flag, relax_timer, relax_countdown, closing_window_flag
    # prompt the option of saving the data
    log_data_flag = False    
    closing_window_flag = True
    # print("check")
    if relax_timer is not None:
        relax_timer.cancel()
        relax_countdown = default_relax_countdown
        relax_timer = None
    if tkMessageBox.askyesno('Save and Quit', 'Do you want to save the result?'):
        generate_file_path()
    
    # if not needed, close window
    else:
        if tkMessageBox.askokcancel('Quit', 'Do you want to quit?'):
            window.destroy()
            rospy.signal_shutdown('Shutting down ROS.')
        else:
            log_data_flag = True
            closing_window_flag = False



# audio to be played for different actions
def audio(filename):
    audio_dir = os.path.realpath(__file__)      # location of the audio files
    audio_dir = audio_dir.replace("tracing_test/scripts/tracing_test.py", "audio/")
    play = Thread( target = playsound, args = ( audio_dir + filename, ) )
    play.start()



if __name__=="__main__":
    # Setup ros related parameters
    global available_time, text_color, window
    rospy.init_node('tracing_test', anonymous = True)
    rospy.Subscriber("/arduino/joystick", Point, joystickCallback)
    
    available_time = rospy.get_param("~timeout_duration", 5)
    text_color = rospy.get_param("~text_color", "black")
    rospy.Timer(rospy.Duration(available_time), timerCallback)

    print("\nAvailable time set to: " + str(available_time) + "s")

    # Generate TKinter window
    w, h, center_x, center_y = generate_window()
    window.update_idletasks()
    time.sleep(3)
    audio('ready.mp3')
    time.sleep(3)
    generate_trace()
    audio('go.mp3')
    rate = rospy.Rate(20)

    update_vertex_cursor = False
    pre_vertic_len = 0

    while not rospy.is_shutdown():  
        move_cursor(moving_distance)
        cursor_position = canvas.coords(target_cursor)
        generate_trace(update = True)
        window.protocol("WM_DELETE_WINDOW", closing_window)
        if len(square_vertices) != pre_vertic_len:
            update_vertex_cursor = True
        else:
            update_vertex_cursor = False
        # print(update_vertex_cursor)
        if (update_vertex_cursor):
            if len(square_vertices) == 5:
                item_list = canvas.find_withtag('vertice_cursor')
                canvas.delete(item_list)
                x_coord, y_coord = fixed_vertices[trial_number - 1][1]
                vertice_cursor = canvas.create_oval((x_coord + distance_threshold), (y_coord + distance_threshold) , (x_coord - distance_threshold) , (y_coord - distance_threshold) , fill='green2', tag='vertice_cursor')

            elif len(square_vertices) == 4:
                item_list = canvas.find_withtag('vertice_cursor')
                canvas.delete(item_list)
                x_coord, y_coord = fixed_vertices[trial_number - 1][2]
                vertice_cursor = canvas.create_oval( (x_coord + distance_threshold) , (y_coord + distance_threshold) , (x_coord - distance_threshold) , (y_coord - distance_threshold) , fill='green2', tag='vertice_cursor')

            elif len(square_vertices) == 3:
                item_list = canvas.find_withtag('vertice_cursor')
                canvas.delete(item_list)
                x_coord, y_coord = fixed_vertices[trial_number - 1][3]
                vertice_cursor = canvas.create_oval( (x_coord + distance_threshold) , (y_coord + distance_threshold) , (x_coord - distance_threshold) , (y_coord - distance_threshold) , fill='green2', tag='vertice_cursor')

            elif len(square_vertices) == 2:
                item_list = canvas.find_withtag('vertice_cursor')
                canvas.delete(item_list)
                x_coord, y_coord = fixed_vertices[trial_number - 1][0]
                vertice_cursor = canvas.create_oval( (x_coord + distance_threshold) , (y_coord + distance_threshold) , (x_coord - distance_threshold) , (y_coord - distance_threshold) , fill='green2', tag='vertice_cursor')

            elif len(square_vertices) == 1:
                item_list = canvas.find_withtag('vertice_cursor')
                canvas.delete(item_list)
        
        item_list = canvas.find_withtag('vertice_cursor')
        canvas.tag_raise(item_list)
        canvas.tag_raise(target_cursor)
        pre_vertic_len = len(square_vertices)
        if len(square_vertices) > 1:
            dist = sqrt((cursor_position[0] - square_vertices[1][0]) ** 2 + (cursor_position[1] - square_vertices[1][1]) ** 2)
            if(dist < distance_threshold):
                square_vertices.pop(0)

                # Update trace once a vertex has been reached
                generate_trace(update = True)



            # start relax countdown once trial successfully completed
            if len(square_vertices) - 1 == 0:
                # print("square",len(square_vertices))
                audio('relax.mp3')
                trial_number = trial_number + 1
                if relax_timer is None:
                    relax_timer = Timer(1.0, relax_tick)
                    relax_timer.start()

            # Project user's pose onto current reference line segment to get reference pose
            curr_user_pose = ((cursor_position[0] + cursor_position[2])/2, (cursor_position[1] + cursor_position[3]) / 2)

            ref_x = 0
            ref_y = 0
            try:
                ref_line_grad = (square_vertices[0][1] - square_vertices[1][1]) / (square_vertices[0][0] - square_vertices[1][0])
                ref_c = (-ref_line_grad * (square_vertices[0][0])) + square_vertices[0][1] # y = mx + c ==> -mx + y = c

                if ref_line_grad != 0:
                    user_grad = -1 / ref_line_grad

                    # y = mx + c ==> -mx + y = c
                    user_c = (-user_grad * (curr_user_pose[0])) + curr_user_pose[1]

                    # x = (c1 - c2) / (m2 - m1)
                    ref_x = (ref_c - user_c) / (user_grad - ref_line_grad)

                # Reference line segment is horizontal
                else:
                    ref_x = curr_user_pose[0]
                    
                # If intersection point is outside of reference line segment, set it to be the nearest line segment
                if ref_x > max(square_vertices[0][0], square_vertices[1][0]):
                    ref_x = max(square_vertices[0][0], square_vertices[1][0])

                if ref_x < min(square_vertices[0][0], square_vertices[1][0]):
                    ref_x = min(square_vertices[0][0], square_vertices[1][0])

                ref_y = ref_line_grad * ref_x + ref_c
            
            # Gradient is infinity
            except:
                ref_x = square_vertices[0][0]
                ref_y = curr_user_pose[1]

                if len(square_vertices) > 1:
                    if curr_user_pose[1] > max(square_vertices[0][1], square_vertices[1][1]):
                        ref_y = max(square_vertices[0][1], square_vertices[1][1])

                    elif curr_user_pose[1] < min(square_vertices[0][1], square_vertices[1][1]):
                        ref_y = min(square_vertices[0][1], square_vertices[1][1])

            # Append logging data
            if log_data_flag:
                trial.append(trial_number)
                # hard coding to get the vertices of the shape as a reference point
                if len(square_vertices) == 5:
                    reference_point.append(fixed_vertices[trial_number - 1][1])
                if len(square_vertices) == 4:
                    reference_point.append(fixed_vertices[trial_number - 1][2])
                if len(square_vertices) == 3:
                    reference_point.append(fixed_vertices[trial_number - 1][3])
                if len(square_vertices) == 2:
                    reference_point.append(fixed_vertices[trial_number - 1][0])
                # print(trial_number)
                # print(len(square_vertices))
                user_poses.append(curr_user_pose)
                user_time.append(time.time() - time_start)
                user_input.append((input_x, input_y))
                raw_user_input.append((raw_x, raw_y))
                reference_poses.append((ref_x, ref_y))


        # generate new polygon once intertrial break finish
        if relax_countdown_flag:

            generate_trace(update = False)
            relax_countdown_flag = False


        # If timer update or all vertices are complete, update trace line
        if update or len(square_vertices) == 1:
            generate_trace(update=True)
            update = False


        # window.protocol("WM_DELETE_WINDOW", closing_window)
        window.update_idletasks()
        window.update()
        rate.sleep()