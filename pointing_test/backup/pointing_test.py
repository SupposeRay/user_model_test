#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import random
import rospy
import math
from geometry_msgs.msg import Point
import os, sys
#  select, termios, tty
import Tkinter as tk
import tkMessageBox
from math import pi
# import subprocess
from playsound import playsound
from threading import Timer, Thread, Event, current_thread
# import getch
import keyboard
import numpy as np


''' 
Moving Cursor refers to the white cursor that will be moved by the joystick.
Target Cursor refers to the yellow cursor each subject should aim to reach.
Success Countdown refers to the amount of time where moving cursor overlaps the target cursor in order for the trial to be considered a success.
Timeout Countdown refers to the amount of time given for the moving cursor to reach the target cursor, otherwise considered failed trial.
Relax Countdown refers to the amount of time break given between end of a success trial and before the start of the next trial.

Different flags during trials:
    1: Still in the start position
    2: Start to move, leaving the start
    3: Within the overlapping target area
    4: Already in the target (flag=3) but you again move out
    5: Successful/failed, then return to start, generate next target
'''



input_x = 0.0
input_y = 0.0
raw_x = 0.0
raw_y = 0.0


# set up interface
# w, h = 1024, 768
Hz = 20     # ROS refresh rate
moving_cursor_r = 10      # size of moving cursor
timer_fontsize = 20     # font of the countdown timer
moving_distance = 50

# TODO:    What's the difference between Moving_distance and big_r

# set up of different time counters
default_success_countdown = 10       # time taken to consider succeeded
default_timeout_countdown = 5       # time taken to consider failed
default_relax_countdown = 5     # break time after each success trial .............. intertrial duration


# set up of different distance criteria
big_r = 100     # distance between moving cusor and target cursor
repeat = 2      # how many times to repeat the trials in each direction


acceptable_range = big_r * 0.25      # considered overlap if within this range (25% of distance between starting point of moving cursor and target cursor)
target_cursor_r = moving_cursor_r * 1.5 # size of target cursor twice of moving cursor

# set up flags
success_timer = None
timeout_timer = None
relax_timer = None
success_countdown = default_success_countdown
timeout_countdown = default_timeout_countdown
relax_countdown = default_relax_countdown
success_countdown_updated = False
timeout_countdown_updated = False
success_countdown_flag = False
timeout_countdown_flag = False
relax_countdown_flag = False
saving_window_flag = False
  


# Logging lists
trial = []      # trial number
flag = []       # different phase in trial
trial_number = 1
flag_number = 1     # flag = 1 ---> at starting position
file_dir = ""       # location of file to be saved at
file_name = ""      # name of the file to be created for logging purposes
user_poses = []     # coordinates of the moving cursor
reference_poses = []        # coordinates of the target cursor
user_time = []      # timeframe at which the data is being captured
user_input = []     # linear velocity and angle velocity of the joystick (with reference to the previous timeframe)
raw_user_input = []



# input from joystick
def joystickCallback(Point):
    global input_x, input_y, raw_x, raw_y
    raw_x = Point.y
    raw_y = Point.x
    if (abs(Point.x) >= 0.04):
        input_y = Point.x
    else:
        input_y = 0
    if (abs(Point.y) >= 0.04):
        input_x = Point.y
    else:
        input_x = 0
    # input_y = Point.x
    # input_x = Point.y



# interact with joystick
def joystick_listener():
    rospy.init_node('pointing_test', anonymous = True)
    rospy.Subscriber("/arduino/joystick", Point, joystickCallback)



# Data to be saved
def save_logs():
    global file_dir_entry, file_name_entry, label_file_dir, label_file_name, save_button, saving_window_flag
    # get information from generate_file_path to get file name and file directory
    file_dir = file_dir_entry.get()
    file_name = file_name_entry.get()

    if len(file_dir) == 0 or len(file_name) == 0:
        print("Please input a full save directory (including last '/') and file name with extension")
        return

    print("Saving logs to " + file_dir + file_name + "_point.txt")
    # trial,flag,ref.x,ref.y,ref.z,ref.o.x,ref.o.y,ref.o.z,ref.o.w,user.time,user.x,user.y,user.z,user.o.x,user.o.y,user.o.z,user.o.w,user.linvel,user.angvel,robot.linvel,robot.angvel
    f = open(file_dir + file_name + "_point.txt", "w")
    if f.closed:
        print("Could not open file to save logs")
        return

    f.writelines("trial,flag,ref.x,ref.y,ref.z,ref.o.x,ref.o.y,ref.o.z,ref.o.w,user.time,user.x,user.y,user.z,user.o.x,user.o.y,\
                user.o.z,user.o.w,user.linvel,user.angvel,raw.linvel,raw.angvel\n")

    for i in range(len(user_poses)):
        save_string = "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                        trial[i],
                        flag[i],
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
                        -raw_user_input[i][1] if len(raw_user_input) > i else "nan")
        
        f.writelines(save_string)

    f.close()
    print("Sucessfully saved to logs")

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
            saving_window_flag = False
        


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



# clear log list if trials are to be recorded after one another
def clear_log_lists():
    global user_poses, reference_poses, user_time, user_input, raw_user_input
    user_poses = []
    reference_poses = []
    user_time = []
    user_input = []
    raw_user_input = []
    flag = []
    trial = []



# set up GUI 
def generate_window():
    "set-up interface, w: width of display, h: height of display"
    global window
    w, h = window.winfo_screenwidth(), window.winfo_screenheight()    # width and height of screen
    center_x, center_y = (w/2),(h/2)    # set up middle of interface  
    window.overrideredirect(False)       # do not show tool bar
    window.geometry("{0}x{1}".format(w,h))
    window.bind('<Escape>', lambda event: closing_window())
    window.title('Pointing Test')
    # interface appearance
    # window_interface = str(w) + 'x' + str(h)
    # window.geometry(window_interface)
    canvas = tk.Canvas(window, bg='black', highlightthickness=5, highlightbackground="black", height=h, width=w)
    canvas.place(x=0, y=0, anchor='nw')
    return canvas, w, h, center_x, center_y



# [x] window
def closing_window():
    global saving_window_flag
    # prompt the option of saving the data
    if tkMessageBox.askyesno('Save and Quit', 'Do you want to save the result?'):
        saving_window_flag = True
        generate_file_path()
    
    # if not needed, close window
    else:
        if tkMessageBox.askokcancel('Quit', 'Do you want to quit?'):
            window.destroy()
            rospy.signal_shutdown('Shutting down ROS.')



# create cursor (either target/moving cursor)
def draw_oval(canvas, x, y, r, color):
    return canvas.create_oval(x - r, y - r, x + r, y + r, fill=color)



def target_list(repeat):
    mylist = []
    # There are 8 target locations.
    for i in np.arange(1, ((8 * repeat) + 1) ):
        # There are 3 target distance.
        for j in np.arange(1,4):
            mylist.append([i,j])
    np.random.shuffle(mylist)   # Note this function is immutable?
    print("Preparing target location done!")
    return mylist



# create target cursors
def target_position(big_r, shuffle, r, w, h):
    global acceptable_range
    n = shuffle[trial_number - 1]
    # get the numbers from the shuffled list
    i = n[0]
    j = n[1]
    dx, dy = math.cos((2 * pi/8) * i) * (big_r * j), math.sin((2 * pi/8) * i) * (big_r * j)
    target_x = (w / 2) + dx
    target_y = (h / 2) + dy
    target_cursor = draw_oval(canvas, target_x, target_y, r, 'yellow')
    acceptable_range = (big_r * j) * 0.25
    return target_cursor, acceptable_range




# set up movement of the moving cursor, based on moving_distance
def move_cursor(canvas):
    "set-up movement of cursors, moving_distance: maximum displacement from the starting point of moving cursor "
    global input_x, input_y, center_x, center_y, moving_distance
    moving_position = canvas.coords(moving_cursor)

    moving_cursor_x = (moving_position[0] + moving_position[2])/2
    moving_cursor_y = (moving_position[1] + moving_position[3])/2

    # set starting position
    x_diff = (center_x) - moving_cursor_x
    y_diff = (center_y) - moving_cursor_y

    # moving_dist = math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2))
    spring_factor = float(moving_distance) / 400
    x_spring = x_diff * spring_factor
    y_spring = y_diff * spring_factor

    # movable distance for moving cursor
    x_move = (- moving_distance) * input_x
    y_move = (- moving_distance) * input_y
    
    x_speed = x_spring + x_move
    y_speed = y_spring + y_move
    canvas.move(moving_cursor, x_speed, y_speed)

        

# calculating the distance beween the center of moving cursor and target cursor, for setting up conditional statement
def overlap(x1, y1, x2, y2, acceptable_range):
    "check whether the distance between center of moving cursor and target cursor is within cutoff"
    return (x1 - x2)**2 + (y1 - y2)**2 <= acceptable_range**2
    


# check if joystick is moving
def failed_trial():
    global input_x, input_y, success_countdown, relax_countdown
    if abs(input_x) > 0.1  or abs(input_y) > 0.1:
        result = 'failed'
    else:
        result = 'not_moving'
        
    return result



# if success condition fulfilled: run this function to start counting down
def success_tick():
    global success_countdown, success_timer, success_countdown_updated, success_countdown_flag, relax_countdown_flag
    success_countdown = success_countdown - 1
    success_countdown_updated = True
    if success_countdown: # t > 0
        success_timer = Timer(1.0, success_tick)
        success_timer.start()
    

    else:  # success t = 0
        success_countdown_flag = True
        success_timer = None
        success_countdown = default_success_countdown
        


    
# if failed condition fulfilled: run this function to start counting down
def timeout_tick():
    global timeout_countdown, timeout_timer, timeout_countdown_updated, timeout_countdown_flag
    timeout_countdown = timeout_countdown - 1
    timeout_countdown_updated = True
    if timeout_countdown:   # start of failed trial
        timeout_timer = Timer(1.0, timeout_tick)
        timeout_timer.start()


    else:   # if failed
        timeout_countdown_flag = True
        timeout_timer = None
        timeout_countdown = default_timeout_countdown



# subject to rest after each successful trial
def relax_tick():
    global relax_countdown, relax_timer, relax_countdown_flag, timeout_timer, success_timer, timeout_countdown, success_countdown, timeout_countdown_updated, success_countdown_updated
    relax_countdown = relax_countdown - 1
    result = failed_trial()
    # flag = 4 ---> restarting trial
    flag_number = 5 

    # stop any timer countdown if triggered
    if timeout_timer is not None:
        timeout_timer.cancel()
        timeout_countdown = default_timeout_countdown
        timeout_timer = None
        timeout_countdown_updated = False
    if success_timer is not None:
        success_timer.cancel()
        success_countdown = default_success_countdown
        success_timer = None
        success_countdown_updated = False
    
    # moving cursor have to at resting position otherwise new target cursor will not be generated
    if not saving_window_flag:
        if result == 'failed':
            relax_countdown = default_relax_countdown
            relax_timer = Timer(1.0, relax_tick)
            relax_timer.start()
   
        # once moving cursor is at resting position, start to countdown the intertrial timer
        else:
            if relax_countdown:
                relax_timer = Timer(1.0, relax_tick)
                relax_timer.start()
                
            else: # done resting
                relax_countdown_flag = True
                relax_timer = None
                relax_countdown = default_relax_countdown
    


# audio to be played for different actions
def audio(filename):
    audio_dir = os.path.realpath(__file__)      # location of the audio files
    audio_dir = audio_dir.replace("pointing_test/scripts/pointing_test.py", "audio/")
    play = Thread( target = playsound, args = ( audio_dir + filename, ) )
    play.start()



if __name__=="__main__":
    # generate GUI
    joystick_listener()    
    window = tk.Tk()
    canvas, w, h, center_x, center_y = generate_window()
    rate = rospy.Rate(Hz)
    moving_cursor = draw_oval(canvas, center_x, center_y, moving_cursor_r, 'white')

    # set up timer display
    button_label = tk.StringVar()
    prompt  = tk.Label(window, textvariable = button_label, font=('Times', timer_fontsize, 'bold'), width=15, height=3, wraplength = 1000, justify = 'left', anchor = 'w', fg='white', background = 'black')
    prompt.place(x=10, y = h - 200) 

    # run GUI with audio
    window.update_idletasks()
    time.sleep(3)
    audio('ready.mp3')
    time.sleep(3)

    # generate random list
    shuffle = target_list(repeat)

    # log timeframe
    start = time.time()
    
    # start trial as soon as target cursor appears
    target_cursor, acceptable_range = target_position(big_r, shuffle, target_cursor_r, w, h)
    audio('go.mp3')

    while not rospy.is_shutdown():
        w, h = window.winfo_screenwidth(), window.winfo_screenheight()
        window.protocol("WM_DELETE_WINDOW", closing_window)
        window.update_idletasks()


        # setting moving cursor to always be on top of target cursor
        canvas.tag_raise(moving_cursor)


        # start moving cursor
        move_cursor(canvas)
        

        # coordinates of the center point of moving cursor
        x1, y1, _, __ = canvas.coords(moving_cursor)
        a = canvas.coords(moving_cursor)
        moving_cursor_position = ((a[0] + a[2])/2, (a[1] + a[3])/2)

        # coordinates of the center point of target cursor
        x2, y2, _, __ = canvas.coords(target_cursor)
        b = canvas.coords(target_cursor)
        target_cursor_position = ((b[0] + b[2])/2, (b[1] + b[3])/2) 


        # moving cursor not overlapping target cursor
        if not overlap(moving_cursor_position[0], moving_cursor_position[1], target_cursor_position[0], target_cursor_position[1], acceptable_range):
            # if trial already started (movement detected from joystick)
            result = failed_trial()
            if result == 'failed':
                
                # flag = 2 ---> does not overlap
                if flag_number == 1:
                    flag_number = flag_number + 1
                # flag = 3 ---> after moment of overlapping
                elif flag_number == 3:
                    flag_number = flag_number + 1  

                # stop success timer if it is running
                if success_timer is not None:
                    success_timer.cancel()
                    success_countdown = default_success_countdown
                    success_timer = None
                    success_countdown_updated = False

                if not saving_window_flag:
                    # start timeout countdown if not already started
                    if timeout_timer is None:
                        timeout_timer = Timer(1.0, timeout_tick)
                        timeout_timer.start()
                
                elif saving_window_flag:
                    # do not start any timer at the saving page
                    if timeout_timer is not None:
                        timeout_timer.cancel()
                        timeout_countdown = default_timeout_countdown
                        timeout_timer = None
                        timeout_countdown_updated = False


        # if moving cursor and target cursor overlap within a range
        else:

            # flag = 3 ---> overlapping
            if flag_number == 2:
                flag_number = flag_number + 1
            elif flag_number == 4:
                flag_number = flag_number - 1

            # stop timeout timer if it is running
            if timeout_timer is not None:
                timeout_timer.cancel()
                timeout_countdown = default_timeout_countdown
                timeout_timer = None
                timeout_countdown_updated = False
            
            if not saving_window_flag:
                # start success timer if not already started
                if success_timer is None:
                    success_timer = Timer(1.0, success_tick)
                    success_timer.start()

            elif saving_window_flag:    
                # do not start any timer at the saving page
                if success_timer is not None:
                    success_timer.cancel()
                    success_countdown = default_success_countdown
                    success_timer = None
                    success_countdown_updated = False


        # timer to start displaying
        if success_countdown_updated:       # show time left to complete a success trial
            button_label.set(success_countdown)
        
        elif timeout_countdown_updated:     # show time left to conclude trial as a failed trial
            button_label.set(timeout_countdown)
        

        # if succeeded
        if success_countdown_flag or timeout_countdown_flag:
            # hide target cursor 
            canvas.itemconfigure(target_cursor, state = 'hidden')
            audio('relax.mp3')

            # flag = 5 ---> intermission after success/failed trial
            if flag_number == 3:
                flag_number = flag_number + 2
            elif flag_number == 4:
                flag_number = flag_number + 1
            
            # stop timeout counter if starts counting after succeed trial
            if timeout_timer is not None:
                timeout_timer.cancel()
                timeout_countdown = default_timeout_countdown
                timeout_timer = None
                timeout_countdown_updated = False

            if success_timer is not None:
                success_timer.cancel()
                success_countdown = default_success_countdown
                success_timer = None
                success_countdown_updated = False

            # prompt saving window page if number of assigned trials completed
            if len(shuffle) == trial_number:
                saving_window_flag = True
                generate_file_path()
                success_countdown_flag = False
                timeout_countdown_flag = False


            else:
                # and start relax counter if it has not already started
                if relax_timer is None:
                    relax_timer = Timer(1.0, relax_tick)
                    relax_timer.start()
                success_countdown_flag = False
                timeout_countdown_flag = False
                

        
        # intermission after each success/failed trial
        if relax_countdown_flag:
            # set trial number and flag number
            trial_number = trial_number + 1
            print("Next trial: ", trial_number)

            # delete cursor and generate next target cursor
            canvas.delete(target_cursor)
            target_cursor, acceptable_range = target_position(big_r, shuffle, target_cursor_r, w, h)
            canvas.itemconfigure(target_cursor, state = 'normal')
            audio('go.mp3')
            relax_countdown_flag = False

            # flag = 1 ---> start position
            flag_number = 1

        if saving_window_flag:
            # stop all timer at the saving page
            if timeout_timer is not None:
                timeout_timer.cancel()
                timeout_countdown = default_timeout_countdown
                timeout_timer = None
                timeout_countdown_updated = False

            if success_timer is not None:
                success_timer.cancel()
                success_countdown = default_success_countdown
                success_timer = None
                success_countdown_updated = False

            if relax_timer is not None:
                relax_timer.cancel()
                relax_countdown = default_relax_countdown
                relax_timer = None

            
            


        # log timeframe
        stop = time.time()


        # add into logging list before the end of each timeframe
        trial.append(trial_number)
        # print(trial_number)
        flag.append(flag_number)
        user_poses.append(moving_cursor_position)
        reference_poses.append(target_cursor_position)
        user_time.append(stop - start)
        user_input.append((input_x, input_y))
        raw_user_input.append((raw_x, raw_y))


        window.update()
        


        rate.sleep()    