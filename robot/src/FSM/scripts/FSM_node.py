#!/usr/bin/python3

from __future__ import print_function

import sys
import rospy
from FSM.srv import *
from FSM.planner import segment_image, plan
from sensor_msgs.msg import Image
from enum import Enum
from cv_bridge import CvBridge
import numpy as np

X_AXIS_LEN = 80 # mm
Y_AXIS_LEN = 170 # mm
EXPLORE_RANGE = 20 # mm 

def sendCommand(clamp, carriage, toolhead, stage1, needle):
    rospy.wait_for_service('Command')
    try:
        commandHandle = rospy.ServiceProxy('Command', Command)
        # NOTE: carriage is inverted due to current movement range in [0,170]
        resp = commandHandle(clamp, -carriage, toolhead, stage1, needle)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

class Robot():
    def __init__(self):
        self.clamp = 0 
        self.carriage = 0
        self.toolhead = 0
        self.stage1 = 0
        self.needle = 0

class State(Enum):
    START = 0
    CLAMP = 1
    EXPLORE = 2
    SEEK = 3
    INJECT = 4
    END = 5

def clamp_logic():
    # Clamp down the arm, reset the explored array
    global robot
    rospy.loginfo("CLAMP")
    clamp = 1
    carriage = robot.carriage
    toolhead = robot.toolhead
    stage1 = 0
    needle = 0
    error = sendCommand(clamp, carriage, toolhead, stage1, needle)
    if error:
        assert False

    # Track robot state
    robot.clamp = 1

    # reset the x exploration
    global x_explored
    x_explored[:] = 0

    # Mark y explored
    global y_explored
    y_pos = robot.carriage
    # Mark a certain range explored
    explored_low = max(y_pos - EXPLORE_RANGE, 0)
    explored_high = min(y_pos + EXPLORE_RANGE, Y_AXIS_LEN)

    y_explored[explored_low : explored_high] = 1
    return State.EXPLORE

def explore_logic(image):
    # Locate vein
    rospy.loginfo("EXPLORE")
    located = len(segment_image(image)) > 0

    # If located: go to SEEK
    if located:
        rospy.loginfo("Found vein. Going to SEEK.")
        return State.SEEK

    # Else continue
    global x_explored
    global robot
    global y_explored

    # if somewhere still unexplored
    if np.count_nonzero(x_explored == 0):
        rospy.loginfo("No vein found. Traversing...")
        
        # move to the nearest unexplored location
        unexplored = np.nonzero(x_explored == 0)[0]
        distance = unexplored - robot.toolhead
        min_index= np.argmin(distance)
        target = unexplored[min_index]

        clamp = 1
        carriage = robot.carriage
        toolhead = target
        stage1 = 0
        needle = 0
        error = sendCommand(clamp, carriage, toolhead, stage1, needle)
        if error:
            assert False
        robot.toolhead = target

        # Mark a certain range explored
        explored_low = max(target - EXPLORE_RANGE, 0)
        explored_high = min(target + EXPLORE_RANGE, X_AXIS_LEN - 1)
        rospy.loginfo(f"X exploration array is of type {type(x_explored)}")
        x_explored[explored_low : explored_high] = 1
        return State.EXPLORE

    else: # this clamp arc completely explored: time to move to a different y
        rospy.loginfo("Arc completely explored. Moving carriage...")

        # safety checks
        assert robot.clamp, "Unclamped in EXPLORE"  # must be clamped
        assert not robot.stage1, "Trying to change y while stage 1 is down"  # stage 1 must be up
        assert not robot.needle, "Trying to change y while needle is injected"  # needle must be retracted
        
        # reset the toolhead
        clamp = 1
        carriage = robot.carriage
        toolhead = 0
        stage1 = 0
        needle = 0
        error = sendCommand(clamp, carriage, toolhead, stage1, needle)
        if error:
            assert False
        robot.toolhead = 0

        # Unclamp
        clamp = 0
        error = sendCommand(clamp, carriage, toolhead, stage1, needle)
        if error:
            assert False
        robot.clamp = 0

        # Y-explore logic
        
        if np.count_nonzero(y_explored == 0):
            # move to the nearest unexplored location
            unexplored = np.nonzero(x_explored == 0)
            distance = unexplored - robot.carriage
            min_index= np.argmin(distance)
            y_target = unexplored[min_index]

            clamp = 0
            carriage = y_target
            toolhead = robot.toolhead
            stage1 = 0
            needle = 0
            error = sendCommand(clamp, carriage, toolhead, stage1, needle)
            if error:
                assert False
            robot.carriage = y_target
            
            assert not robot.clamp, "Trying to clamp when already clamped."
            return State.CLAMP

        else: # all the y positions exhausted
            return State.END # end in failure
            # alternatively, can re-explore

def seek_logic(image):
    rospy.loginfo("SEEK")
    pixel_vector = plan(image)

    if abs(pixel_vector) > 640: # this means that no veins were found
        rospy.loginfo("No valid veins found. Going to EXPLORE")
        return State.EXPLORE

    # go from pixel distance to stepper command 
    # 100 mm -> 640 pixels
    distance_vector = pixel_vector * 100 / 640 # mm

    if distance_vector < 0.5:
        rospy.loginfo("Close enough to vein! Going to INJECT")
        return State.INJECT
    # wheel is about 16 mm
    else:
        rospy.loginfo("Homing in on vein...")
        global robot
        clamp = 1
        carriage = robot.carriage
        toolhead = robot.toolhead + distance_vector
        stage1 = 0
        needle = 0
        error = sendCommand(clamp, carriage, toolhead, stage1, needle)
        if error:
            assert False
        robot.toolhead += distance_vector

def inject_logic():
    rospy.loginfo("INJECT")
    global robot
    clamp = 1
    carriage = robot.carriage
    toolhead = robot.toolhead
    stage1 = 1
    needle = 0

    # Drop stage1
    error = sendCommand(clamp, carriage, toolhead, stage1, needle)
    if error:
        assert False
    robot.stage1 = 1
    
    # Inject Needle
    needle = 1
    error = sendCommand(clamp, carriage, toolhead, stage1, needle)
    if error:
        assert False
    robot.needle = 1
    return State.END

def end_logic():
    pass # right now do nothing, we want to check where the needle lands
    '''
    # Raise the needle, then stage 1
    clamp = 1
    carriage = robot.carriage
    toolhead = robot.toolhead
    stage1 = 1
    needle = 0

    # Retract needle
    error = sendCommand(clamp, carriage, toolhead, stage1, needle)
    if error:
        assert False
    robot.needle = 0

    # Retract stage 1
    stage1 = 0
    error = sendCommand(clamp, carriage, toolhead, stage1, needle)
    if error:
        assert False
    robot.stage1 =0
    '''

def spin_FSM(segmented_image):
    image = bridge.imgmsg_to_cv2(segmented_image)
    assert image.shape == (480, 640), f"Image message was {image.shape}"
    # TODO: get image into binary format: 0 or 1
    global state
    global robot
    if state == State.START:
        # reset y explored and go to CLAMP
        y_explored[:] = 0
        state = State.CLAMP

    elif state == State.CLAMP:
        result_state = clamp_logic()
        state = result_state

    elif state == State.EXPLORE:
        assert robot.clamp, "Exploring while unclamped"
        assert not robot.stage1, "Exploring with stage 1 down"
        assert not robot.needle, "Exploring with needle injecting"
        
        result_state = explore_logic(image) 
        state = result_state

    elif state == State.SEEK:
        assert robot.clamp, "Seeking while unclamped"
        assert not robot.stage1, "Seeking with stage 1 down"
        assert not robot.needle, "Seeking with needle injecting"
        result_state = seek_logic(image)
        state == result_state

    elif state == State.INJECT:
        result_state = inject_logic()
        # state transition: go to END. Wait for visual assessment
        state = result_state

    elif state == State.END:
        end_logic()
    else:
        assert 0, "State machine is in incorrect state! Terminating..."

if __name__ == "__main__":
    robot = Robot()
    state = State.START

    y_explored = np.zeros((Y_AXIS_LEN,))
    x_explored = np.zeros((X_AXIS_LEN,))

    bridge = CvBridge()
    rospy.loginfo("Initializing FSM")
    rospy.init_node('FSM', anonymous = True)
    #spin_FSM() # spin FSM once to get out of start state
    #spin_FSM() # spin again to get into clamp
    rospy.Subscriber('segmented_images', Image, spin_FSM)
    rospy.loginfo(f"FSM ready to go: state = {state}")
    rospy.spin()
    

