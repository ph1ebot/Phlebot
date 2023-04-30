#!/usr/bin/env python
#import rospy
#from phlebot_command import Command
import time
from commands import *

class RobotState():
    def __init__(self):
        self.clamped = False
        self.carriage_position = 0
        self.carriage_direction = 1 # or -1
        self.toolhead_position = 0
        self.stage1 = False # 0 for up, 1 for down
        self.needle = False # 0 for retracted, 1 for injected
'''
def callback(command):
    # decompose into component commands

    # Send clamping first
    clamp_command = command.clamp
    if clamp_command != robot.clamped:
        if clamp_command:
            clamp_arm()
        else:
            raise_clamping_arm()
        robot.clamped = not robot.clamped
        rospy.sleep(CLAMP_PERIOD)

    # Send carriage next (absolute position)
    carriage_command = command.carriage
    if carriage_command != robot.carriage_position:
        if not robot.clamped: # not carriage movement during clamping
            move_carriage(carriage_command)
            robot.carriage_position = carriage_command
            rospy.sleep(CARRIAGE_PERIOD)

    # Send toolhead (delta position)
    toolhead_command = command.toolhead
    if toolhead_command:
        new_position = toolhead_command + robot.toolhead_position
        move_toolhead(new_position)
        robot.toolhead_position = new_position
        rospy.sleep(TOOLHEAD_PERIOD)

    # send stage 1
    stage1_command = command.stage1
    if stage1_command != robot.stage1:
        send_gcode(STAGE_ONE_DOWN)
        robot.stage1 = stage1_command
        rospy.sleep(STAGE1_PERIOD)

    # send needle
    needle_command = command.needle
    if needle_command != robot.needle:
        if robot.stage1: # stage 1 veto
            send_gcode(INJECT)
            robot.needle = needle_command
            rospy.sleep(NEEDLE_PERIOD)
    
    rospy.loginfo(rospy.get_caller_id() + " send a command to klipper")
'''
def exercise():
    print("Trying carriage")
    send_gcode(move_carriage(-80))
    #rospy.sleep(80 * 0.05)
    time.sleep(80 * 0.4)
    print("Trying toolhead")
    send_gcode(move_toolhead(30))
    #rospy.sleep(50 * 0.5)
    time.sleep(30 * 0.4)
    print("Returning carriage")
    send_gcode(move_carriage(0))
    #rospy.sleep(80 * 0.05)
    time.sleep(80 * 0.4)
    print("Returning toolhead")
    send_gcode(move_toolhead(0))
    #rospy.sleep(50 * 0.5)
    time.sleep(30 * 0.4)
    return


if __name__ == '__main__':
    connect()
    #rospy.sleep(0.5)
    #sleep(0.5)
    print("Homing...")

    home()
    home_stepper()
    print("Done homing")

    robot = RobotState()

    exercise()

    #rospy.init_node('listener', anonymous=True)

    #rospy.Subscriber("commands", Command, callback)

    #rospy.spin()

