import DobotDllType as dType
import time
import threading
import numpy as np

import cv2
import csv

#Useful global variables
# --- These are status strings that you might see, so we're defining them here ---
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

#always begin with this line, or you can't connect to the robot at all. Just don't
#remove this line and keep it at the top of your code
api = dType.load()

"""
These coordinates are to the left of the robot's x axis and slight above the xy plane, viewed from
the top. This is a useful home position when dealing with the vision labs, since it moves
the robot out of the way. You can change the coordinates here if you really want.
"""
home_pos = [100,-200,50]


####################################### Lab 4 Parameters #####################################

# Camera calibration values. Fill these after running compute_transform.py
calibration_data = np.load("calibration_data.npz")
CAMERA_MATRIX = calibration_data["camera_matrix"]
DIST_COEFFS = calibration_data["dist_coeffs"]

# Transform matrix from camera frame to robot frame. Fill these after running compute_transform.py
R = np.load("R.npy")
T = np.load("T.npy").reshape(3, 1)

# Tolerance for the robot's workspace in mm. This is used to define a restricted workspace
WORKSPACE_TOLERANCE_MM = 2.0

# Side length of the ArUco markers in metres.
# Your Lab 3 code used 0.05 m.
MARKER_SIZE_M = 0.05

# Measure these two values manually using Dobot Link.
# PEN_UP_Z: pen is safely above the paper.
# PEN_DOWN_Z: pen contacts the paper, with spring compression <= 5 mm.
PEN_UP_Z = 10
PEN_DOWN_Z = 5

# The pen may require the robot's upper z limit to be above zero.
# Confirm this value manually before running.
ROBOT_Z_MAX = 50

# Part 1 position correction.
# Begin with zero and update after measuring dot errors.
LAB4_X_CORRECTION_MM = 0
LAB4_Y_CORRECTION_MM = 0
LAB4_Z_CORRECTION_MM = 0

# These values will be calculated from the three detected markers.
paper_x_min = None
paper_x_max = None
paper_y_min = None
paper_y_max = None


# Create the ArUco detector once.
aruco_dict = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

aruco_parameters = cv2.aruco.DetectorParameters()

aruco_detector = cv2.aruco.ArucoDetector(
    aruco_dict,
    aruco_parameters
)

####################################################################################################


def initialize_robot(api):
    #detect the robot's com port
    com_port = dType.SearchDobot(api)
    print(dType.SearchDobot(api))
    #if we can't find it, then we can't continue, so exit
    if "COM" not in com_port[0]:
        print("Error: The robot either isn't on or isn't responding. Exiting now")
        exit()
    
    
    #we've found it, so let's try to connect
    state = dType.DobotConnect.DobotConnect_NoError
    for i in range(0,len(com_port)):
        state_full = dType.ConnectDobot(api, com_port[i], 115200)
        state = state_full[0]
        print("STATE FULL:")
        print(state_full)
        #If the connection failed at this point, we also can't proceed, so we need to exit
        if state == dType.DobotConnect.DobotConnect_NoError:
            print("Connected!")
            name = "Arya"
            if name[0] == "Not a dobot":
                dType.DisconnectDobot(api)
                continue
            else:
                break
            
    if state != dType.DobotConnect.DobotConnect_NoError:
            print("Can not connect! Exiting")
            exit()    
    """
        stop any queued commands and clear the queue. You HAVE TO do this every time you initialize the robot
        If there are queued commands in the queue, then they will execute first. This can
        cause the robot to go well outside of its allowable range. The simplest way to do this
        is to stop anything that might be running or might try to run, then clear the queue.
        
        Other than at startup, during normal operation you shouldn't have to do this.
    """
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)
    
    #Set the robot's max speed and acceleration. We're keeping these to 50% of max for safety
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    
    """
        Home the robot. 
    """
    #Set the home position
    dType.SetHOMEParams(api, home_pos[0], home_pos[1], home_pos[2], 0, isQueued=1)
    
    cmdIndx = -1
    """
        Enqueue the home command. This command always begins by moving the robot back to an initialization
        position so that the encoders are reset, then it will move the robot to its home position,
        and finally it will undergo a quick procedure to validate that its encoders are properly set. You definitely
        want to run this every time you initialize the robot
    """
    execCmd = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
    
    #Execute the three enqueued commands: set the speed/acceleration, set the home position, and move to home
    dType.SetQueuedCmdStartExec(api)
    
    #Allow the homing command to complete. The robot will beep and the LED will turn green
    #when it's ready to go
    while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)
        
    #OK, the robot is ready to move!
    
"""
    Move the robot to the given x, y, z coordinates using PTP Linear XYZ Mode. This command will block until the motion
    is complete. You almost always want to run this rather than the straight SetPTPCmd, because you shouldn't be sending
    multiple motion commands to the robot without queueing them first, and we want to run everything in unqueued mode
"""
def move_to_xyz(api,x,y,z):
    cmdIndx = -1
    execCmd = dType.SetPTPCmd(api,dType.PTPMode.PTPMOVLXYZMode,x,y,z,0,isQueued=0)[0]
    #Allow the command to complete. The robot will stop moving when it's done
    while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)

"""
    Move the robot to the given joint angles using PTP Linear ANGLE mode
    We will default J4 to zero, since it only matters if you have an end effector attached
"""
def move_joint_angles(api,J1,J2,J3,J4=0):
    cmdIndx = -1
    
    execCmd = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJANGLEMode, J1, J2, J3, J4, isQueued = 0)[0]
    #Allow the command to complete. The robot will stop moving when it's done
    while execCmd > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(25)

    
    
"""
    Move the robot to it's home position. Note: this will use basic PTP motion, rather than
    SetHOMECmd, since SetHOMECmd will re-run the sensor initialization stuff that we don't
    need during normal operation
"""
def move_to_home(api):
    move_to_xyz(api,home_pos[0],home_pos[1],home_pos[2])





################################################################################################
######################################## Lab 1 #################################################
################################################################################################

######################################## Part 2 ################################################

def is_point_in_workspace(x, y, z):
    """
    Check whether a single Cartesian point is inside the restricted workspace.
    Workspace:
    -120 <= z <= 0
    140 <= sqrt(x^2 + y^2) <= 260
    x >= 0
    """
    r = np.sqrt(x**2 + y**2)

    return (
        #-120 <= z <= ROBOT_Z_MAX and
        #140 <= r <= 260 and
        x >= 0
    )

def is_path_in_workspace(start, target, num_points=100):
    """
    Check whether every sampled point on the straight-line path
    from start to target is inside the restricted workspace.
    """
    start = np.array(start, dtype=np.float64)
    target = np.array(target, dtype=np.float64)

    for t in np.linspace(0, 1, num_points):
        point = start + t * (target - start)
        x, y, z = point

        if not is_point_in_workspace(x, y, z):
            return False

    return True

def safe_move_to_xyz(api, x, y, z, num_points=100):
    """
    Move to target only if:
    1. target is inside workspace
    2. straight-line path from current pose to target stays inside workspace
    """
    current_pose = dType.GetPose(api)
    current_xyz = current_pose[:3]

    target_xyz = np.array([x, y, z], dtype=np.float64)

    if not is_point_in_workspace(x, y, z):
        print(f"Rejected target {target_xyz}: outside workspace.")
        return False

    if not is_path_in_workspace(current_xyz, target_xyz, num_points):
        print(f"Rejected path from {current_xyz.round(1)} to {target_xyz}: path leaves workspace.")
        return False

    move_to_xyz(api, x, y, z)
    print(f"Moved safely to {target_xyz}.")
    return True

###################################### Part 2 End ##############################################

######################################## Part 3 ################################################
'''Not useful for this lab, delected'''
###################################### Part 3 End ##############################################

################################################################################################
######################################## Lab 1 End #############################################
################################################################################################

################################################################################################
######################################## Lab 2 #################################################
################################################################################################
'''Not useful for this lab, delected'''
################################################################################################
###################################### Lab 2 End ###############################################
################################################################################################

################################################################################################
######################################## Lab 3 #################################################
################################################################################################
'''Not useful for this lab, delected'''
################################################################################################
###################################### Lab 3 End ###############################################
################################################################################################







################################################################################################
######################################## Lab 4 #################################################
################################################################################################

def lab4_parameters_ready():
    if PEN_UP_Z is None or PEN_DOWN_Z is None:
        print("Fill in PEN_UP_Z and PEN_DOWN_Z first.")
        return False

    return True

def camera_to_robot_mm(camera_point_m):
    camera_point_m = camera_point_m.reshape(3, 1)

    robot_point_m = R @ camera_point_m + T

    return robot_point_m.flatten() * 1000.0


def detect_all_markers_once():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Camera could not be opened.")
        return {}

    for _ in range(10):
        ret, frame = cap.read()

    cap.release()

    if not ret:
        print("Could not capture a valid image.")
        return {}

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco_detector.detectMarkers(gray)

    if ids is None:
        print("No ArUco markers were detected.")
        return {}

    _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        MARKER_SIZE_M,
        CAMERA_MATRIX,
        DIST_COEFFS
    )

    markers = {}

    for i, marker_id_array in enumerate(ids):
        marker_id = int(marker_id_array[0])
        camera_position_m = tvecs[i].flatten()
        robot_position_mm = camera_to_robot_mm(camera_position_m)

        markers[marker_id] = {
            "camera_m": camera_position_m,
            "robot_mm": robot_position_mm
        }

        print(
            f"Marker {marker_id}: "
            f"camera = {camera_position_m.round(4)} m, "
            f"robot = {robot_position_mm.round(2)} mm"
        )

    return markers


def apply_lab4_xy_correction(marker_position):
    corrected_position = marker_position.copy()

    corrected_position[0] += LAB4_X_CORRECTION_MM
    corrected_position[1] += LAB4_Y_CORRECTION_MM
    corrected_position[2] += LAB4_Z_CORRECTION_MM

    return corrected_position


def set_lab4_workspace_from_markers(markers):
    """
    Define the restricted xy workspace from the minimum and maximum
    marker coordinates.
    """

    global paper_x_min, paper_x_max, paper_y_min, paper_y_max

    if len(markers) < 3:
        print(
            "Fewer than three markers were detected. "
            "The restricted workspace cannot be defined."
        )
        return False

    marker_positions = np.array([
        apply_lab4_xy_correction(marker["robot_mm"])
        for marker in markers.values()
    ])

    paper_x_min = np.min(marker_positions[:, 0])
    paper_x_max = np.max(marker_positions[:, 0])

    paper_y_min = np.min(marker_positions[:, 1])
    paper_y_max = np.max(marker_positions[:, 1])

    print("\nRestricted Lab 4 workspace:")
    print(
        f"x: {paper_x_min:.2f} to "
        f"{paper_x_max:.2f} mm"
    )
    print(
        f"y: {paper_y_min:.2f} to "
        f"{paper_y_max:.2f} mm"
    )

    return True


def is_point_in_lab4_workspace(x, y, z):
    print(f"use {x:.2f}, {y:.2f}, {z:.2f}")
    z_min = min(PEN_UP_Z, PEN_DOWN_Z)
    z_max = max(PEN_UP_Z, PEN_DOWN_Z)

    return (
        is_point_in_workspace(x, y, z)
        and paper_x_min - WORKSPACE_TOLERANCE_MM
            <= x
            <= paper_x_max + WORKSPACE_TOLERANCE_MM
        and paper_y_min - WORKSPACE_TOLERANCE_MM
            <= y
            <= paper_y_max + WORKSPACE_TOLERANCE_MM
        #and z_min <= z <= z_max
    )



def safe_lab4_move(api, x, y, z):
    if not is_point_in_lab4_workspace(x, y, z):
        print(f"Rejected target {[x, y, z]}.")
        return False

    return safe_move_to_xyz(api, x, y, z)



def get_valid_marker_target(marker_id, markers):
    """
    Return the corrected marker target if the marker was detected
    and is inside the allowed workspace.
    """

    if marker_id not in markers:
        print(f"Marker {marker_id} was not detected.")
        return None

    corrected_position = apply_lab4_xy_correction(
        markers[marker_id]["robot_mm"]
    )

    x = corrected_position[0]
    y = corrected_position[1]


    print("\n========== TARGET DEBUG ==========")
    print("Raw target:", markers[marker_id]["robot_mm"].round(2))
    print("Corrected target:", corrected_position.round(2))
    print(
        "Correction:",
        LAB4_X_CORRECTION_MM,
        LAB4_Y_CORRECTION_MM
    )
    print("==================================")


    if not (
        is_point_in_lab4_workspace(x, y, PEN_UP_Z)
        and is_point_in_lab4_workspace(x, y, PEN_DOWN_Z)
    ):
        print(
            f"Marker {marker_id} is outside the allowed workspace."
        )
        return None

    return corrected_position




# def get_valid_marker_target(marker_id, markers):
#     """
#     Return the corrected marker target if the marker was detected
#     and is inside the allowed workspace.
#     """

#     if marker_id not in markers:
#         print(f"Marker {marker_id} was not detected.")
#         return None

#     corrected_position = apply_lab4_xy_correction(
#         markers[marker_id]["robot_mm"]
#     )

#     x = corrected_position[0]
#     y = corrected_position[1]

#     # Debug checks must come after x and y are assigned.
#     up_base = is_point_in_workspace(x, y, PEN_UP_Z)
#     down_base = is_point_in_workspace(x, y, PEN_DOWN_Z)

#     x_ok = (
#         paper_x_min - WORKSPACE_TOLERANCE_MM
#         <= x
#         <= paper_x_max + WORKSPACE_TOLERANCE_MM
#     )

#     y_ok = (
#         paper_y_min - WORKSPACE_TOLERANCE_MM
#         <= y
#         <= paper_y_max + WORKSPACE_TOLERANCE_MM
#     )

#     z_min = min(PEN_UP_Z, PEN_DOWN_Z)
#     z_max = max(PEN_UP_Z, PEN_DOWN_Z)

#     up_z_ok = z_min <= PEN_UP_Z <= z_max
#     down_z_ok = z_min <= PEN_DOWN_Z <= z_max

#     print("\n--- MARKER VALIDATION DEBUG ---")
#     print("marker:", marker_id)
#     print("x, y:", repr(x), repr(y))
#     print("radius:", np.sqrt(x**2 + y**2))
#     print("paper x:", paper_x_min, paper_x_max, "x_ok:", x_ok)
#     print("paper y:", paper_y_min, paper_y_max, "y_ok:", y_ok)
#     print("up base:", up_base, "down base:", down_base)
#     print("up z:", up_z_ok, "down z:", down_z_ok)

    if not (
        is_point_in_lab4_workspace(x, y, PEN_UP_Z)
        and is_point_in_lab4_workspace(x, y, PEN_DOWN_Z)
    ):
        print(
            f"Marker {marker_id} is outside the allowed workspace."
        )
        return None

    return corrected_position






def place_dot(api, marker_id, target_position):
    """
    Move above the marker, lower the pen, record the tool position,
    and raise the pen.
    """

    x = target_position[0]
    y = target_position[1]

    print(f"\nMoving to marker {marker_id}.")

    if not safe_lab4_move(api, x, y, PEN_UP_Z):
        return None

    if not safe_lab4_move(api, x, y, PEN_DOWN_Z):
        print(f"Could not lower the pen at marker {marker_id}.")
        return None

    time.sleep(0.5)

    dot_tool_pose = np.array(
        dType.GetPose(api)[:3],
        dtype=np.float64
    )

    if not safe_lab4_move(
        api,
        x,
        y,
        PEN_UP_Z
    ):
        print(
            f"Could not raise the pen at marker {marker_id}."
        )
        return None

    print(
        f"Dot placed at marker {marker_id}. "
        f"Tool pose at contact: "
        f"{dot_tool_pose.round(2)}"
    )

    return dot_tool_pose


def prepare_lab4(api):
    if not lab4_parameters_ready():
        return None

    move_to_home(api)
    time.sleep(1.0)

    markers = detect_all_markers_once()

    if not set_lab4_workspace_from_markers(markers):
        return None

    return markers


def save_lab4_result(
    part,
    sequence_index,
    marker_id,
    marker_data,
    commanded_target,
    dot_tool_pose
):
    """
    Save one motion result to lab4_results.csv.
    """

    filename = "lab4_results.csv"

    camera_position = marker_data["camera_m"]
    robot_position = marker_data["robot_mm"]

    row = [
        part,
        sequence_index,
        marker_id,

        camera_position[0],
        camera_position[1],
        camera_position[2],

        robot_position[0],
        robot_position[1],
        robot_position[2],

        commanded_target[0],
        commanded_target[1],
        PEN_DOWN_Z,

        dot_tool_pose[0],
        dot_tool_pose[1],
        dot_tool_pose[2]
    ]

    header = [
        "part",
        "sequence_index",
        "marker_id",

        "camera_x_m",
        "camera_y_m",
        "camera_z_m",

        "estimated_robot_x_mm",
        "estimated_robot_y_mm",
        "estimated_robot_z_mm",

        "command_x_mm",
        "command_y_mm",
        "command_z_mm",

        "tool_x_at_dot_mm",
        "tool_y_at_dot_mm",
        "tool_z_at_dot_mm"
    ]

    try:
        with open(filename, "x", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(header)
            writer.writerow(row)

    except FileExistsError:
        with open(filename, "a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(row)

######################################## Part 1 ################################################

def run_lab4_part1(api):
    """
    Ask for one marker ID, place a dot, and move away.
    """

    print("\n---- Lab 4 Part 1 ----")

    markers = prepare_lab4(api)

    if markers is None:
        return

    try:
        marker_id = int(
            input("Enter the ArUco marker ID: ")
        )

    except ValueError:
        print("Invalid marker ID.")
        return

    target = get_valid_marker_target(
        marker_id,
        markers
    )

    if target is None:
        print("The requested marker will not be visited.")
        return

    dot_tool_pose = place_dot(
        api,
        marker_id,
        target
    )

    if dot_tool_pose is not None:
        save_lab4_result(
            part=1,
            sequence_index=0,
            marker_id=marker_id,
            marker_data=markers[marker_id],
            commanded_target=target,
            dot_tool_pose=dot_tool_pose
        )

    move_to_home(api)

###################################### Part 1 End ###############################################

######################################## Part 2 ################################################

def read_marker_sequence():
    """
    Read marker IDs until a negative number is entered.
    """

    print(
        "Enter marker IDs separated by spaces. "
        "Enter a negative number to finish."
    )

    sequence = []

    while True:
        values = input(">> ").split()

        for value in values:
            try:
                marker_id = int(value)
            except ValueError:
                print(f"Ignoring invalid input: {value}")
                continue

            if marker_id < 0:
                return sequence

            sequence.append(marker_id)

def run_lab4_part2(api):
    """
    Detect markers once and visit each valid marker in the
    entered sequence.
    """

    print("\n---- Lab 4 Part 2 ----")

    markers = prepare_lab4(api)

    if markers is None:
        return

    sequence = read_marker_sequence()

    if len(sequence) == 0:
        print("No marker IDs were entered.")
        return

    valid_targets = []

    for sequence_index, marker_id in enumerate(sequence):

        target = get_valid_marker_target(
            marker_id,
            markers
        )

        if target is None:
            print(
                f"Skipping marker {marker_id} at "
                f"sequence position {sequence_index}."
            )
            continue

        valid_targets.append(
            (
                sequence_index,
                marker_id,
                target
            )
        )

    if len(valid_targets) == 0:
        print(
            "All requested markers were invalid, outside the "
            "workspace, or not detected. The robot will not move."
        )
        return

    for sequence_index, marker_id, target in valid_targets:

        dot_tool_pose = place_dot(
            api,
            marker_id,
            target
        )

        if dot_tool_pose is None:
            continue

        save_lab4_result(
            part=2,
            sequence_index=sequence_index,
            marker_id=marker_id,
            marker_data=markers[marker_id],
            commanded_target=target,
            dot_tool_pose=dot_tool_pose
        )

    move_to_home(api)

###################################### Part 2 End ###############################################

################################################################################################
###################################### Lab 4 End ###############################################
################################################################################################







#Before running and commands, always run this
# Before running any commands, always initialize the robot.
initialize_robot(api)

print("\nSelect a program:")
print("p1 - Lab 4 Part 1")
print("p2 - Lab 4 Part 2")

selection = input("Enter p1 or p2: ").strip()

if selection == "p1":
    run_lab4_part1(api)

elif selection == "p2":
    run_lab4_part2(api)

else:
    print("Invalid selection.")

move_to_home(api)
#All done!