#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Point
# import sys, select, termios, tty
import Tkinter as tk
import tkMessageBox

input_x = 0.0
input_y = 0.0
window = tk.Tk()

def joystickCallback(Point):
    global input_x, input_y
    input_y = Point.x
    input_x = Point.y
    # rospy.loginfo('User input x = %f, y = %f', input_x, input_y)


def joystick_listener():
    rospy.init_node('pointing_test', anonymous = True)
    rospy.Subscriber("/keyboard", Point, joystickCallback)

def generate_window():
    global window, target_cursor, moving_cursor, trajectory, speed, stop_cmd
    stop_cmd = 0
    speed = 0
    trajectory = 0
    window.title('Tracking Test')
    window.geometry('1024x768')    
    canvas = tk.Canvas(window, bg='sky blue', highlightthickness=3, \
        highlightbackground="black", height=600, width=800)
    target_cursor = canvas.create_oval(395, 295, 405, 305, fill='black', tag='cursor')
    moving_cursor = canvas.create_oval(395, 295, 405, 305, fill='red', tag='mover')
    def generate_trace(tag):
        item_list = canvas.find_withtag('trace')
        canvas.delete(item_list)
        item_list = canvas.find_withtag('cursor')
        canvas.delete(item_list)
        item_list = canvas.find_withtag('mover')
        canvas.delete(item_list)
        global target_cursor, moving_cursor, trajectory, speed
        if tag == 1:
            trace = canvas.create_rectangle(150, 50, 650, 550, outline='blue', width=1, tag='trace')
            target_cursor = canvas.create_oval(145, 295, 155, 305, fill='black', tag='cursor')
            moving_cursor = canvas.create_oval(145, 295, 155, 305, fill='red', tag='mover')
            trajectory = 1
            speed = 0
        elif tag == 2:
            trace = canvas.create_rectangle(275, 175, 525, 425, outline='blue', width=1, tag='trace')
            target_cursor = canvas.create_oval(270, 295, 280, 305, fill='black', tag='cursor')
            moving_cursor = canvas.create_oval(270, 295, 280, 305, fill='red', tag='mover')
            trajectory = 2
            speed = 0
        elif tag == 3:
            trace = canvas.create_oval(150, 50, 650, 550, outline='blue', width=1, tag='trace')
            target_cursor = canvas.create_oval(145, 295, 155, 305, fill='black', tag='cursor')
            moving_cursor = canvas.create_oval(145, 295, 155, 305, fill='red', tag='mover')
            trajectory = 3
            speed = 0
        elif tag == 4:
            trace = canvas.create_oval(275, 175, 525, 425, outline='blue', width=1, tag='trace')
            target_cursor = canvas.create_oval(270, 295, 280, 305, fill='black', tag='cursor')
            moving_cursor = canvas.create_oval(270, 295, 280, 305, fill='red', tag='mover')
            trajectory = 4
            speed = 0
        else:
            pass

    def move_cursor():
        global moving_cursor, trajectory, speed, stop_cmd
        mover_position = canvas.coords(moving_cursor)
        mover_x = (mover_position[0] + mover_position[2])/2
        mover_y = (mover_position[1] + mover_position[3])/2
        if trajectory == 1:# large square
            if speed == 1:# fast
                if (mover_x == 150) and (mover_y <= 550) and (mover_y > 50):
                    if mover_y - 50 >= 10:
                        canvas.move(moving_cursor, 0, -10)
                    else:
                        canvas.move(moving_cursor, 0, 50 - mover_y)
                if (mover_y == 50) and (mover_x >= 150) and (mover_x < 650):
                    if 650 - mover_x >= 10:
                        canvas.move(moving_cursor, 10, 0)
                    else:
                        canvas.move(moving_cursor, 650 - mover_x, 0)
                if (mover_x == 650) and (mover_y >= 50) and (mover_y < 550):
                    if 550 - mover_y >= 10:
                        canvas.move(moving_cursor, 0, 10)
                    else:
                        canvas.move(moving_cursor, 0, 550 - mover_y)
                if (mover_y == 550) and (mover_x <= 650) and (mover_x > 150):
                    if mover_x - 150 >= 10:
                        canvas.move(moving_cursor, -10, 0)
                    else:
                        canvas.move(moving_cursor, 150 - mover_x, 0)
                return
            elif speed == 2:# slow
                if (mover_x == 150) and (mover_y > 50) and (mover_y <= 550):
                    canvas.move(moving_cursor, 0, -5)
                if (mover_y == 50) and (mover_x >= 150) and (mover_x < 650):
                    canvas.move(moving_cursor, 5, 0)
                if (mover_x == 650) and (mover_y >= 50) and (mover_y < 550):
                    canvas.move(moving_cursor, 0, 5)
                if (mover_y == 550) and (mover_x <= 650) and (mover_x > 150):
                    canvas.move(moving_cursor, -5, 0)
                return
        if trajectory == 2:# small square
            if speed == 1:# fast
                if (mover_x == 275) and (mover_y <= 425) and (mover_y > 175):
                    if mover_y - 175 >= 10:
                        canvas.move(moving_cursor, 0, -10)
                    else:
                        canvas.move(moving_cursor, 0, 175 - mover_y)
                if (mover_y == 175) and (mover_x >= 275) and (mover_x < 525):
                    if 525 - mover_x >= 10:
                        canvas.move(moving_cursor, 10, 0)
                    else:
                        canvas.move(moving_cursor, 525 - mover_x, 0)
                if (mover_x == 525) and (mover_y >= 175) and (mover_y < 425):
                    if 425 - mover_y >= 10:
                        canvas.move(moving_cursor, 0, 10)
                    else:
                        canvas.move(moving_cursor, 0, 425 - mover_y)
                if (mover_y == 425) and (mover_x <= 525) and (mover_x > 275):
                    if mover_x - 275 >= 10:
                        canvas.move(moving_cursor, -10, 0)
                    else:
                        canvas.move(moving_cursor, 275 - mover_x, 0)
                return
            elif speed == 2:# slow
                if (mover_x == 275) and (mover_y > 175) and (mover_y <= 425):
                    canvas.move(moving_cursor, 0, -5)
                if (mover_y == 175) and (mover_x >= 275) and (mover_x < 525):
                    canvas.move(moving_cursor, 5, 0)
                if (mover_x == 525) and (mover_y >= 175) and (mover_y < 425):
                    canvas.move(moving_cursor, 0, 5)
                if (mover_y == 425) and (mover_x <= 525) and (mover_x > 275):
                    canvas.move(moving_cursor, -5, 0)
                return
        vector_x = 400 - mover_x
        vector_y = 300 - mover_y
        if trajectory == 3:# large cricle
            r_circle = 250
            acos_value = math.acos(vector_x / r_circle)
            current_angle = acos_value if vector_y >= 0 else math.pi * 2 - acos_value
            if speed == 1:# fast
                current_angle = current_angle + math.pi / 48
                current_angle = current_angle if current_angle < math.pi * 2 else current_angle - math.pi * 2
                vector_x = r_circle * math.cos(current_angle)
                vector_y = r_circle * math.sin(current_angle)
                next_x = 400 - vector_x
                next_y = 300 - vector_y
                canvas.move(moving_cursor, next_x - mover_x, next_y - mover_y)
                return
            elif speed == 2:# slow
                current_angle = current_angle + math.pi / 96
                current_angle = current_angle if current_angle < math.pi * 2 else current_angle - math.pi * 2
                vector_x = r_circle * math.cos(current_angle)
                vector_y = r_circle * math.sin(current_angle)
                next_x = 400 - vector_x
                next_y = 300 - vector_y
                canvas.move(moving_cursor, next_x - mover_x, next_y - mover_y)
                return
        if trajectory == 4:# small circle
            r_circle = 125
            acos_value = math.acos(vector_x / r_circle)
            current_angle = acos_value if vector_y >= 0 else math.pi * 2 - acos_value
            if speed == 1:# fast
                current_angle = current_angle + math.pi / 48
                current_angle = current_angle if current_angle < math.pi * 2 else current_angle - math.pi * 2
                vector_x = r_circle * math.cos(current_angle)
                vector_y = r_circle * math.sin(current_angle)
                next_x = 400 - vector_x
                next_y = 300 - vector_y
                canvas.move(moving_cursor, next_x - mover_x, next_y - mover_y)
                return
            elif speed == 2:# slow
                current_angle = current_angle + math.pi / 96
                current_angle = current_angle if current_angle < math.pi * 2 else current_angle - math.pi * 2
                vector_x = r_circle * math.cos(current_angle)
                vector_y = r_circle * math.sin(current_angle)
                next_x = 400 - vector_x
                next_y = 300 - vector_y
                canvas.move(moving_cursor, next_x - mover_x, next_y - mover_y)

    def set_speed(tag):
        global speed
        speed = tag
    prompt = tk.Label(window, text='Please use the black dot to track the red dot for 1 lap.',\
        font=('Times', 20, 'bold'), width=70, height=3, wraplength = 1000,\
        justify = 'left', anchor = 'w', fg='black')
    prompt.place(x=10, y = 650)

    canvas.place(x=0, y=0, anchor='nw')

    button1 = tk.Button(window, text='Large Square',font=('Times', 20), command=lambda tag = 1: generate_trace(tag)).place(x=825, y=20)
    button2 = tk.Button(window, text='Small Square',font=('Times', 20), command=lambda tag = 2: generate_trace(tag)).place(x=825, y=80)
    button3 = tk.Button(window, text='Large Circle',font=('Times', 20), command=lambda tag = 3: generate_trace(tag)).place(x=825, y=140)
    button4 = tk.Button(window, text='Small Circle',font=('Times', 20), command=lambda tag = 4: generate_trace(tag)).place(x=825, y=200)
    button5 = tk.Button(window, text='Fast',font=('Times', 20), command=lambda tag = 1: set_speed(tag)).place(x=825, y=400)
    button6 = tk.Button(window, text='Slow',font=('Times', 20), command=lambda tag = 2: set_speed(tag)).place(x=825, y=500)
    button7 = tk.Button(window, fg='red', text='Stop',font=('Times', 20, 'bold'), command=lambda tag = 0: set_speed(tag)).place(x=825, y=600)

    def move_target():
        global input_x, input_y, target_cursor

        x_speed = -10 * input_x
        y_speed = -10 * input_y

        target_position = canvas.coords(target_cursor)

        if (target_position[0] <= 0) and (x_speed < 0):
            x_speed = 0
        if (target_position[2] >= 800) and (x_speed > 0):
            x_speed = 0
        if (target_position[1] <= 0) and (y_speed < 0):
            y_speed = 0
        if (target_position[3] >= 600) and (y_speed > 0):
            y_speed = 0

        canvas.move(target_cursor, x_speed, y_speed)
    
    def closing_window():
        if tkMessageBox.askokcancel('Quit', 'Do you want to quit?'):
            window.destroy()
            rospy.signal_shutdown('Shutting down ROS.')
    # def timerCallback(event):
    #     if not rospy.is_shutdown():
    #         move_target()
    #         move_cursor()
    #         window.update_idletasks()
    #         window.update()

    # rospy.Timer(rospy.Duration(0.05), timerCallback)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        move_target()
        move_cursor()
        window.protocol("WM_DELETE_WINDOW", closing_window)
        window.update_idletasks()
        window.update()
        rate.sleep()


if __name__=="__main__":
    joystick_listener()
    generate_window()