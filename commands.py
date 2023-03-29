import requests
import time

# GCODE Constants
CLAMP_ARM = "SET_FAN_SPEED FAN=fan1 SPEED=125"
OPEN_CLAMP = "SET_FAN_SPEED FAN=fan1 SPEED=0"

STAGE_ONE_DOWN = "SET_FAN_SPEED FAN=fan2 SPEED=125"
STAGE_ONE_UP = "SET_FAN_SPEED FAN=fan2 SPEED=0"

INJECT = "M106 S125"
RETRACT = "M106 S0"

SPEED_GCODE_Y = " F200"

GET_POS = "M114"
HOME = "G28"

INCREMENT_MODE = "G91"
ABSOLUTE_MODE = "G90"


# For traveling to injection position
TRAVEL_TIME = 3
# For deploying stage
DEPLOY_TIME = 0.5


API_KEY = "CF0BF4529D63426B892A227F1F1F2936"

def move_carriage(position):
    return "G0 Y " + str(position) + SPEED_GCODE_Y


def move_toolhead(position):
    return "G0 Z " + str(position) + SPEED_GCODE_Y


def send_gcode(cmd):
    """
    Send G-CODE command to controller using OctoPrint API
    """
    url = 'http://127.0.0.1:5000/api/printer/command'
    myobj = {'command': cmd, 'passive':'true' }
    header = {"content-type" : "application/json", "X-Api-Key":API_KEY}

    x = requests.post(url, headers=header, json=myobj)
    print(x.content)
    return x


def get_position():
    return send_gcode(GET_POS).json

def connect():
    url = 'http://127.0.0.1:5000/api/login'
    myobj = {'passive':'true' }
    header = {"content-type" : "application/json", "X-Api-Key":API_KEY}

    x = requests.post(url, headers=header, json=myobj)
    print(x.content)
    return x


def home_stepper():
    send_gcode(HOME)


def increment_toolhead(distance):
    send_gcode(INCREMENT_MODE)
    send_gcode(move_toolhead(distance))
    send_gcode(ABSOLUTE_MODE)


def increment_carraige(distance):
    send_gcode(INCREMENT_MODE)
    send_gcode(move_carriage(distance))
    send_gcode(ABSOLUTE_MODE)


def home():
    """
    Returns all pneumatics to retracted position and steppers to zero
    """
    # Retract needle and raise injection stage
    retraction()
    # Return camera toolhead to home
    # Return injection toolhead to home


def raise_clamping_arm():
    """
    Actuate solenoid to raise the clamping arm
    """
    send_gcode(OPEN_CLAMP)


def clamp_arm():
    """
    Actuate solenoid to lower the clamping arm
    """
    send_gcode(CLAMP_ARM)


def injection(injection_position):
    """
    Function to complete the steps for deploying the injection stage and inserting the needle
    Args:
        injection_position (float): Absolute position of injection
    """

    # Move injection toolhead
    move_toolhead(injection_position)
    # Allow time to reach destination
    time.sleep(TRAVEL_TIME)
    # Deploy stage
    send_gcode(STAGE_ONE_DOWN)
    # Pause to prevent unwanted simultaneous motion
    time.sleep(DEPLOY_TIME)
    # Actuate insertion needle
    send_gcode(INJECT)


def retraction():
    """
    Function to complete the steps to retract a needle and the injection stage
    """
    # Retract needle
    send_gcode(RETRACT)
    # Pause to prevent unwanted simultaneous motion
    time.sleep(DEPLOY_TIME)
    # Raise Stage
    send_gcode(STAGE_ONE_UP)
