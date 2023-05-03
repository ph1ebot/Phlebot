#!/usr/bin/env python3

from __future__ import print_function

import rospy
from FSM.commands import *
from FSM.srv import Command, CommandResponse


class RobotState():
    def __init__(self):
        self.clamped = False
        self.carriage_position = 0
        self.carriage_direction = 1 # or -1
        self.toolhead_position = 0
        self.stage1 = False # 0 for up, 1 for down
        self.needle = False # 0 for retracted, 1 for injected

CLAMP_PERIOD = 10
CARRIAGE_PERIOD = 0.4
TOOLHEAD_PERIOD = 0.4
STAGE1_PERIOD = 5

def sendCommand(command):
    # decompose into component commands

    # Send clamping first
    clamp_command = command.clamp
    if clamp_command != robot.clamped:
        if clamp_command:
            rospy.loginfo("Sending clamp signal")
            clamp_arm()
        else:
            rospy.loginfo("Sending UNclamp signal")
            raise_clamping_arm()
        robot.clamped = not robot.clamped
        rospy.sleep(CLAMP_PERIOD)

    # Send carriage next (absolute position)
    carriage_command = command.carriage
    if carriage_command != robot.carriage_position:
        if not robot.clamped: # not carriage movement during clamping
            rospy.loginfo("Sending carraige")
            move_carriage(carriage_command)
            delta = abs(carriage_command -  robot.carriage_positions)
            rospy.sleep(CARRIAGE_PERIOD * delta)
            robot.carriage_position = carriage_command
        else:
            rospy.loginfo("VETO Carriage: clamped")

    # Send toolhead (delta position)
    toolhead_command = command.toolhead
    if toolhead_command != robot.toolhead_position:
        rospy.loginfo("Sending toolhead")
        delta = abs(toolhead_command - robot.toolhead_position)
        move_toolhead(toolhead_command)
        robot.toolhead_position = toolhead_command
        rospy.sleep(TOOLHEAD_PERIOD * delta)

    # send stage 1
    stage1_command = command.stage1
    if stage1_command != robot.stage1:
        if stage1_command:
            rospy.loginfo("Sending stage1 down")
            send_gcode(STAGE_ONE_DOWN)
        else:
            rospy.loginfo("Sending stage1 up")
            send_gcode(STAGE_ONE_UP)
        robot.stage1 = stage1_command
        rospy.sleep(STAGE1_PERIOD)

    # send needle
    needle_command = command.needle
    if needle_command != robot.needle:
        if robot.stage1: # stage 1 vetoi
            if needle_command:
                rospy.loginfo("Sending needle inject")
                send_gcode(INJECT)
            else:
                rospy.loginfo("Sending needle retract")
                send_gcode(RETRACT)

            robot.needle = needle_command
            rospy.sleep(NEEDLE_PERIOD)
    
    return 0

def commandServer():
    rospy.init_node('Command_server')
    s = rospy.Service('Command', Command, sendCommand)
    init_motors()
    rospy.loginfo("Ready to send klipper commands")

    rospy.spin()

def exercise():
    send_gcode(RETRACT)
    send_gcode(STAGE_ONE_UP)
    send_gcode(OPEN_CLAMP)
    
    CARRIAGE_TEST_DIST = 20
    TOOLHEAD_TEST_DIST = 10
    
    rospy.loginfo("Trying carriage")
    send_gcode(move_carriage(CARRIAGE_TEST_DIST))
    rospy.sleep(CARRIAGE_TEST_DIST * CARRIAGE_PERIOD)
    
    rospy.loginfo("Trying toolhead")
    send_gcode(move_toolhead(TOOLHEAD_TEST_DIST))
    rospy.sleep(TOOLHEAD_TEST_DIST * TOOLHEAD_PERIOD)
    
    rospy.loginfo("Returning carriage")
    send_gcode(move_carriage(0))
    rospy.sleep(CARRIAGE_TEST_DIST * CARRIAGE_PERIOD)
    
    rospy.loginfo("Returning toolhead")
    send_gcode(move_toolhead(0))
    rospy.sleep(TOOLHEAD_TEST_DIST * TOOLHEAD_PERIOD)
    return

def init_motors():
    rospy.loginfo("Initializing system.")
    #connect()
    rospy.loginfo("Homing...")

    home()
    home_stepper()
    rospy.loginfo("Done homing")

    exercise()

if __name__ == '__main__':

    robot = RobotState()

    commandServer()
