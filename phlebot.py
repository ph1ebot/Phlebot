import commands
from statemachine import StateMachine, State
import time
# States:
# Booting (Transition on all tests finished w/o error)
# Home (Transition on function end)
# Open (Transition on function end)
# Clamped (Transition on function end)
# Searching (Transition on serach failed/ target found)
# Search Failed (Terminal state)
# Retargeting (Transition on function end)
# Injecting (Transition on time)
# Retracting (Transition on function end) -> Home

# GCODE Reference
# G0 Y10 F6000 Moving Y
# SET_FAN_SPEED FAN=fan1 SPEED=0 Closing solenoid SPEED=125 Open
# M106 S125 Open Solenoid on fan1 port


TEST_DELAY = 2

SEARCH_STEP_SIZE = 5
SEARCH_Y_SIZE = 5


class PhlebotStateMachine(StateMachine):
    booting = State(initial=True)
    home = State()
    arm_open = State()
    arm_clamped = State()
    searching = State()
    search_failed = State()
    large_retarget_st = State()
    small_retarget_st = State()
    injecting = State()
    retracting = State()

    post_complete = booting.to(home)
    opening_arm = home.to(arm_open)
    clamping = arm_open.to(arm_clamped)
    searching_loop = searching.to(large_retarget_st, cond="injection_site_found") | \
        searching.to(search_failed, cond="fully_searched")
    retargetting = large_retarget_st.to(small_retarget_st)
    positioning = small_retarget_st.to(injecting)
    injection_completed = injecting.to(retracting)

    def __init__(self):
        self.injection_site_found = False
        self.fully_searched = False


class PhlebotState:
    def __init__(self):
        carriage_pos = -1
        y_pos = -1
        stage_up = True
        needle_retracted = True


def POST():
    """
    Power-On Self Test
    Verifies functionality of various actuators
    """
    # Set current position as home
    commands.home_stepper()
    # Test camera toolhead stepper
    commands.increment_toolhead(5)
    commands.increment_toolhead(-5)
    time.sleep(TEST_DELAY)
    # Test carriage stepper
    commands.increment_carraige(5)
    commands.increment_carraige(-5)
    time.sleep(TEST_DELAY)
    # Test injection toolhead stepper

    # Testing arm solenoid
    commands.clampArm()
    time.sleep(TEST_DELAY)
    commands.raiseClampingArm()
    time.sleep(TEST_DELAY)

    # Test injection solenoid
    commands.send_gcode(commands.INJECT)
    time.sleep(TEST_DELAY)
    commands.send_gcode(commands.RETRACT)
    time.sleep(TEST_DELAY)
    # Test drop solenoid
    commands.send_gcode(commands.STAGE_ONE_DOWN)
    time.sleep(TEST_DELAY)
    commands.send_gcode(commands.STAGE_ONE_UP)
    time.sleep(TEST_DELAY)


def local_search():
    """
    Function to complete tasks for vein search at individual y slice
    """

    # Step along path
    # Take photo
    # Send photo to NN
    # Read confidences and compare for successful identification


def global_search():
    """
    Function to search entire arm for targetable vein
    """

    # Initialise y region
    # Move to initial y
    # Complete local search
    # Check for successful id move to retarget
    # Increement y region, check against limits


def small_retarget():
    """
    Function to retarget step location for injection on local region
    """

    # Record current step location
    # Home at known speed
    # Record time to home
    # Record steps to home
    # Average different predictions to compute step distance


def large_retarget():
    """
    Function to retarget step location for injection on entire map
    """

    # Un distort image
    # Compute pixels in each direction to target
    # Convert pixels to true distance
    # Convert true distance to steps
    # Home Ca
    # Release arm
    # Move Y
    # Re clamp
    # small_retarget


def main():
    phlebot = PhlebotStateMachine()
    POST()
    phlebot.post_complete()
    phlebot.arm_open()
    # TODO: Add feedback to reclamp
    input()
    commands.clamp_arm()
    phlebot.arm_clamped()

    # Searching loop
    while (not phlebot.fully_searched or not phlebot.injection_site_found):
        pass


if __name__ == '__main__':
    main()
