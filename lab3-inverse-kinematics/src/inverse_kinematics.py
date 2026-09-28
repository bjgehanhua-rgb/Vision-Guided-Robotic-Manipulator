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
home_pos = [200,100,50]

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
        -120 <= z <= 0 and
        140 <= r <= 260 and
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

######################################## Part 1 ################################################

Link1 = 135
Link2 = 147
EE_offset = 59.7

def forward_kinematics(J1, J2, J3):
    """
    Calculate end-effector xyz position from J1, J2, J3.
    Input J1, J2, J3 in degree

    Note:
    J1 = pose[4]
    J2 = pose[5]
    J3 = pose[6]
    (J3 is defined as the forearm angle relative to the ground,
     not the joint angle relative to previous link.)
    """
    J1_rad = np.deg2rad(J1)
    J2_rad = np.deg2rad(J2)
    J3_rad = np.deg2rad(J3)

    r = Link1 * np.sin(J2_rad) + Link2 * np.cos(J3_rad) + EE_offset
    z = Link1 * np.cos(J2_rad) - Link2 * np.sin(J3_rad)

    x = r * np.cos(J1_rad)
    y = r * np.sin(J1_rad)

    return np.array([x, y, z], dtype=np.float64)

def is_joint_in_workspace(J1, J2, J3):
    """
    Check whether a joint configuration is valid by converting it to xyz
    and reusing the Lab 1 workspace function.
    """
    x, y, z = forward_kinematics(J1, J2, J3)
    return is_point_in_workspace(x, y, z)


def validate_offline_data(filename):
    """
    Validate FK using Lab2DesignData.txt.
    The file columns are:
    x, y, z, J1, J2, J3
    """

    data = np.loadtxt(filename)

    results = []

    for i, row in enumerate(data):
        actual_x = row[0]
        actual_y = row[1]
        actual_z = row[2]

        J1 = row[3]
        J2 = row[4]
        J3 = row[5]

        actual_xyz = np.array([actual_x, actual_y, actual_z], dtype=np.float64)
        fk_xyz = forward_kinematics(J1, J2, J3)

        error = np.linalg.norm(fk_xyz - actual_xyz)

        results.append([
            i,
            actual_x, actual_y, actual_z,
            fk_xyz[0], fk_xyz[1], fk_xyz[2],
            error
        ])

    results = np.array(results)

    print("\n----- FK Validation -----")
    print("Mean error: ", np.mean(results[:, 7]))
    print("Max error: ",np.max(results[:, 7]))
    print("Min error:  ", np.min(results[:, 7]))
    print("\n")


# filename = r"C:\Users\bjgeh\Desktop\ECE 486\Lab2\Lab2DesignData.txt"
# validate_offline_data(filename)

###################################### Part 1 End ##############################################

######################################## Part 2 ################################################
'''Not useful for this lab, delected'''
###################################### Part 2 End ##############################################

######################################## Part 3 ################################################
'''Not useful for this lab, delected'''
###################################### Part 3 End ##############################################

################################################################################################
###################################### Lab 2 End ###############################################
################################################################################################







################################################################################################
######################################## Lab 3 #################################################
################################################################################################

######################################## Part 1 ################################################
def inverse_kinematics(x, y, z):

    J1 = np.rad2deg(np.arctan2(y, x))

    r = np.sqrt(x**2 + y**2)
    r0 = r - EE_offset

    D = (r0**2 + z**2 - Link1**2 - Link2**2) / (2 * Link1 * Link2)
    D = np.clip(D, -1, 1)

    gamma = np.arcsin(D)

    A = Link1 + Link2 * np.sin(gamma)
    B = Link2 * np.cos(gamma)

    J2_rad = np.arctan2(r0, z) - np.arctan2(B, A)

    FA_rad = J2_rad - gamma

    J2 = np.rad2deg(J2_rad)
    FA = np.rad2deg(FA_rad)

    return J1, J2, FA


def validate_inverse_kinematics(filename):
    """
    Lab 3 Part 1:
    Validate IK using txt file.
    """

    data = np.loadtxt(filename)
    errors = []

    for row in data:
        x, y, z = row[0:3]
        q = row[3:6]

        J1, J2, FA = inverse_kinematics(x, y, z)
        q_ik = np.array([J1, J2, FA])

        error = np.linalg.norm(q_ik - q)
        errors.append(error)

    errors = np.array(errors)

    print("----- IK Validation -----")
    print("Mean Error :", np.mean(errors))
    print("Max Error  :", np.max(errors))
    print("Min Error  :", np.min(errors))
    

#filename = r"C:\Users\bjgeh\Desktop\ECE 486\Lab2\Lab2DesignData.txt"
#validate_inverse_kinematics(filename)

###################################### Part 1 End ###############################################


######################################## Part 2 ################################################

def ik_fk_validation(api):

    print("\n---- IK FK Validation ----")

    test_joints = [[0, 20, 70],
                   [45, 45, 45],
                   [-45, 30, 70],
                   [0, 30, 60],
                   [20, 30, 60],
                   [-20, 30, 60]]

    errors = []

    for i, joints in enumerate(test_joints):

        move_joint_angles(api, *joints, 0)

        pose = dType.GetPose(api)

        actual_xyz = pose[:3]

        actual_J1 = pose[4]
        actual_J2 = pose[5]
        actual_FA = pose[6]

        ik_J1, ik_J2, ik_FA = inverse_kinematics(actual_xyz[0],
                                                 actual_xyz[1],
                                                 actual_xyz[2])

        actual_q = np.array([actual_J1, actual_J2, actual_FA])

        ik_q = np.array([ik_J1, ik_J2, ik_FA])

        error = np.linalg.norm(ik_q - actual_q)

        errors.append(error)

        print(
            f"Point {i}: "
            f"Actual Joints = {actual_q.round(8)}, "
            f"IK Joints = {ik_q.round(8)}, "
            f"Error = {error:.8f} deg"
        )

    errors = np.array(errors)

    print("\n--- IK Validation Summary ---")
    print(f"Mean error: {np.mean(errors):.8f} deg")
    print(f"Max error:  {np.max(errors):.8f} deg")
    print(f"Min error:  {np.min(errors):.8f} deg")

###################################### Part 2 End ###############################################


######################################## Part 3 ################################################

def camera_validation(api):

    print("\n---- Lab 3 Part 3 Validation ----")

    # Fill these after running compute_transform.py
    R = np.array([[-0.60740703, -0.55410305,  0.56923326],
                  [-0.72715913,  0.0993171,  -0.67924643],
                  [ 0.31983792, -0.82650222, -0.463247  ]], dtype=np.float64)

    T = np.array([[-0.23747033], [0.62191486], [0.34901836]], dtype=np.float64)

    # Your camera calibration values
    camera_matrix = np.array([[1.50380025e+03, 0.00000000e+00, 3.13485612e+02],
                              [0.00000000e+00, 3.15898471e+03, 2.40184062e+02],
                              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float32)

    dist_coeffs = np.array([[ 2.04070743e-01],
                            [-4.03613477e+00],
                            [-1.08718487e-02],
                            [-1.07762460e-03],
                            [-1.93417151e+01]], dtype=np.float32)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    validation_points = [
        (160, -30, -10),
        (180, -30, -10),
        (210, -30, -10),
        (240, -30, -10),

        (160, 20, -10),
        (180, 20, -10),
        (210, 20, -10),
        (240, 20, -10),

        (160, 60, -10),
        (180, 60, -10),
        (210, 60, -10),
        (240, 60, -10),

        (250, 20, -20),
        (200, -20, -20),
        (230, -20, -20),

        (170, 30, -20),
        (200, 30, -20),
        (230, 30, -20),

        (180, 50, -30),
        (220, 50, -30)
    ]

    cap = cv2.VideoCapture(0)
    results = []

    for i, point in enumerate(validation_points):

        x, y, z = point

        print(f"\nMoving to point {i}: {point}")
        move_to_xyz(api, x, y, z)
        time.sleep(2.5)

        pose = dType.GetPose(api)
        actual_robot_xyz = np.array(pose[:3], dtype=np.float64)

        # Read a few frames to allow camera image to update
        for _ in range(5):
            ret, frame = cap.read()

        if not ret:
            print("Camera error. Skipping point.")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None:
            print("No ArUco marker detected. Skipping point.")
            continue

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, 0.05, camera_matrix, dist_coeffs
        )

        X_camera = tvecs[0].flatten().reshape(3, 1)

        predicted_robot_m = R @ X_camera + T
        predicted_robot_xyz = predicted_robot_m.flatten() * 1000

        error_xyz = actual_robot_xyz - predicted_robot_xyz
        error_norm = np.linalg.norm(error_xyz)

        results.append([
            i,
            actual_robot_xyz[0], actual_robot_xyz[1], actual_robot_xyz[2],
            predicted_robot_xyz[0], predicted_robot_xyz[1], predicted_robot_xyz[2],
            error_xyz[0], error_xyz[1], error_xyz[2],
            error_norm
        ])

        print("Actual Robot XYZ:", actual_robot_xyz.round(2))
        print("Camera Predicted XYZ:", predicted_robot_xyz.round(2))
        print("Error XYZ:", error_xyz.round(2))
        print("Euclidean Error:", round(error_norm, 2), "mm")

    cap.release()
    cv2.destroyAllWindows()

    if len(results) == 0:
        print("No validation points were recorded.")
        return

    with open("part3_validation_results.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "point",
            "actual_x_mm", "actual_y_mm", "actual_z_mm",
            "pred_x_mm", "pred_y_mm", "pred_z_mm",
            "error_x_mm", "error_y_mm", "error_z_mm",
            "euclidean_error_mm"
        ])
        writer.writerows(results)

    errors = np.array([row[-1] for row in results])

    print("\n----- Part 3 Validation Summary -----")
    print("Number of valid points:", len(errors))
    print("Mean error:", np.mean(errors))
    print("Max error:", np.max(errors))
    print("Min error:", np.min(errors))
    print("RMSE:", np.sqrt(np.mean(errors ** 2)))
    print("Saved to part3_validation_results.csv")

###################################### Part 3 End ###############################################


################################################################################################
###################################### Lab 3 End ###############################################
################################################################################################





#Before running and commands, always run this
initialize_robot(api)

"""
    Here is a sample script that moves the robot to a position, then moves back to home, then to another position, five times
    
    It also prints the pose of the robot. Then, we move the robot by joint angle just to show how it's done.
"""

# explore_workspace_joint_angles(api)
# run_joint_fk_validation(api)

#ik_fk_validation(api)

camera_validation(api)

move_to_home(api)
#All done!