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

# The pen may require the robot's upper z limit to be above zero.
# Confirm this value manually before running.
ROBOT_Z_MAX = 50

# Height of the table/paper plane in the robot coordinate system, in mm.
# Measure this value in the lab and replace None.
GRASP_Z_MM = None

# Safe Z height for horizontal robot movement, in mm.
# Measure this value in the lab and replace None.
HORIZONTAL_MOVE_Z_MM = None

# Fixed angular offset caused by the physical mounting direction
# of the gripper. Determine this experimentally.
GRIPPER_MOUNT_OFFSET_DEG = 0.0

# Empirical correction between the estimated object position
# and the real centre of the gripper, in mm.
GRIPPER_X_CORRECTION_MM = 0.0
GRIPPER_Y_CORRECTION_MM = 0.0

# Pixel distance used when converting an image-space direction
# into a robot-frame direction.
ANGLE_SAMPLE_LENGTH_PX = 50.0

# ArUco marker used as the toy bin.
# Fill this in with the marker ID chosen in the lab.
TOY_BIN_MARKER_ID = None

# Optional XY correction from the marker centre
# to the actual drop location, in mm.
TOY_BIN_X_OFFSET_MM = 0.0
TOY_BIN_Y_OFFSET_MM = 0.0


# Create the ArUco detector once.
aruco_dict = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

aruco_parameters = cv2.aruco.DetectorParameters()

aruco_detector = cv2.aruco.ArucoDetector(
    aruco_dict,
    aruco_parameters
)




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


################################################################################################
######################################## Lab 1 End #############################################
################################################################################################


################################################################################################
######################################## Lab 4 #################################################
################################################################################################


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

################################################################################################
###################################### Lab 4 End ###############################################
################################################################################################



################################################################################################
######################################## Lab 5 #################################################
################################################################################################

def capture_camera_frame(camera_index=0):
    """
    Capture one high-resolution image from the webcam.

    This function only uses the camera.
    It does not connect to or move the robot.
    """

    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("Camera could not be opened.")
        return None

    # Request 1920 x 1080 resolution.
    cap.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG")
    )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    frame = None
    ret = False

    # Read several frames so the camera exposure can stabilize.
    for _ in range(20):
        ret, frame = cap.read()

    cap.release()

    if not ret or frame is None:
        print("Could not capture a valid image.")
        return None

    return frame


def detect_all_coloured_objects(frame):
    """
    Detect all sufficiently colourful objects on a mostly white background.

    Returns:
        detections:
            A list containing information about each object.

        debug_image:
            The original image with boxes and centres drawn.

        colour_mask:
            A black-and-white image where white represents colourful regions.
    """

    # Convert the camera image from BGR to HSV.
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Separate the HSV channels.
    hue, saturation, value = cv2.split(hsv)

    # White paper normally has low saturation.
    # Painted blocks normally have much higher saturation.
    colour_mask = cv2.inRange(
        saturation,
        55,
        255
    )

    # Remove very dark regions such as deep shadows.
    brightness_mask = cv2.inRange(
        value,
        45,
        255
    )

    colour_mask = cv2.bitwise_and(
        colour_mask,
        brightness_mask
    )

    # Remove small noise and fill small holes.
    kernel = np.ones((7, 7), dtype=np.uint8)

    colour_mask = cv2.morphologyEx(
        colour_mask,
        cv2.MORPH_OPEN,
        kernel,
        iterations=1
    )

    colour_mask = cv2.morphologyEx(
        colour_mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=2
    )

    # Find separate connected coloured regions.
    contours, _ = cv2.findContours(
        colour_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    detections = []
    debug_image = frame.copy()

    for contour in contours:
        area = cv2.contourArea(contour)

        # Ignore small coloured noise.
        if area < 1200:
            continue

        # Find the smallest rotated rectangle enclosing the object.
        rect = cv2.minAreaRect(contour)

        center = rect[0]
        size = rect[1]
        angle = rect[2]

        center_x = int(round(center[0]))
        center_y = int(round(center[1]))

        width = size[0]
        height = size[1]

        # Ignore extremely narrow invalid detections.
        if width < 10 or height < 10:
            continue

        # Normalize the angle so it approximately follows the object's long axis.
        if width < height:
            angle += 90

        # Get the four corners of the rotated rectangle.
        box = cv2.boxPoints(rect)
        box = np.int32(np.round(box))

        detections.append({
            "pixel_center": (center_x, center_y),
            "area_px": area,
            "long_axis_angle_deg": float(angle),
            "box": box,
            "contour": contour
        })

        # Draw the object's contour.
        cv2.drawContours(
            debug_image,
            [contour],
            -1,
            (255, 255, 255),
            2
        )

        # Draw the rotated bounding box.
        cv2.polylines(
            debug_image,
            [box],
            True,
            (0, 0, 0),
            3
        )

        # Draw the detected centre.
        cv2.circle(
            debug_image,
            (center_x, center_y),
            6,
            (0, 0, 0),
            -1
        )

        label = (
            f"object ({center_x}, {center_y}) "
            f"angle={angle:.1f}"
        )

        cv2.putText(
            debug_image,
            label,
            (center_x - 80, max(25, center_y - 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 0),
            2
        )

    # Sort objects from top to bottom, then left to right.
    detections.sort(
        key=lambda item: (
            item["pixel_center"][1],
            item["pixel_center"][0]
        )
    )

    return detections, debug_image, colour_mask


def pixel_to_robot_plane(u, v, GRASP_Z_MM):
    """
    Convert an image pixel coordinate (u, v) into a robot-frame
    position on the horizontal plane z = GRASP_Z_MM.

    Parameters
    ----------
    u, v:
        Pixel coordinates in the original camera image.

    GRASP_Z_MM:
        Known height of the table/paper plane in robot coordinates,
        measured in millimetres.

    Returns
    -------
    np.ndarray:
        [robot_x_mm, robot_y_mm, robot_z_mm]
    """

    # Put the pixel into the format expected by cv2.undistortPoints().
    pixel = np.array(
        [[[float(u), float(v)]]],
        dtype=np.float64
    )

    # Correct lens distortion and convert the pixel into normalized
    # camera coordinates.
    undistorted_pixel = cv2.undistortPoints(
        pixel,
        CAMERA_MATRIX,
        DIST_COEFFS
    )

    normalized_x = undistorted_pixel[0, 0, 0]
    normalized_y = undistorted_pixel[0, 0, 1]

    # This is the direction of the ray from the camera centre
    # through the detected image pixel.
    ray_camera = np.array(
        [
            [normalized_x],
            [normalized_y],
            [1.0]
        ],
        dtype=np.float64
    )

    # Rotate the ray direction from the camera frame
    # into the robot frame.
    ray_robot = R @ ray_camera

    # Your calibration equation is:
    #
    # robot_point = R @ camera_point + T
    #
    # Therefore the camera origin expressed in the robot frame is T.
    camera_origin_robot_m = T.reshape(3)

    # Convert table height from millimetres to metres,
    # because R and T use metres.
    table_z_m = float(GRASP_Z_MM) / 1000.0

    ray_robot_z = ray_robot[2, 0]

    if abs(ray_robot_z) < 1e-9:
        raise ValueError(
            "The camera ray is parallel to the table plane."
        )

    # Find where the robot-frame ray reaches z = table_z_m.
    scale = (
        table_z_m - camera_origin_robot_m[2]
    ) / ray_robot_z

    if scale <= 0:
        raise ValueError(
            "The calculated table intersection is behind the camera. "
            "Check R, T, GRASP_Z_MM, and the detected pixel."
        )

    robot_point_m = (
        camera_origin_robot_m
        + scale * ray_robot.reshape(3)
    )

    robot_point_mm = robot_point_m * 1000.0

    # Force the returned Z value to exactly equal the measured plane height.
    robot_point_mm[2] = float(GRASP_Z_MM)

    return robot_point_mm


def normalize_parallel_gripper_angle(angle_deg):
    """
    Normalize an angle to [-90, 90).

    A parallel-jaw gripper rotated by 180 degrees has the same
    grasping orientation.
    """

    angle_deg = float(angle_deg)

    while angle_deg >= 90.0:
        angle_deg -= 180.0

    while angle_deg < -90.0:
        angle_deg += 180.0

    return angle_deg


def image_long_axis_to_robot_angle(
    center_u,
    center_v,
    image_long_axis_angle_deg,
    GRASP_Z_MM,
    sample_length_px=ANGLE_SAMPLE_LENGTH_PX
):
    """
    Convert the object's long-axis direction from image coordinates
    into the robot XY coordinate system.
    """

    theta_image_rad = np.radians(
        float(image_long_axis_angle_deg)
    )

    half_length_px = float(sample_length_px) / 2.0

    delta_u = (
        np.cos(theta_image_rad) * half_length_px
    )

    delta_v = (
        np.sin(theta_image_rad) * half_length_px
    )

    # Construct two image points on opposite sides of the centre,
    # following the detected long axis.
    pixel_1_u = float(center_u) - delta_u
    pixel_1_v = float(center_v) - delta_v

    pixel_2_u = float(center_u) + delta_u
    pixel_2_v = float(center_v) + delta_v

    # Convert both pixels to points on the robot-frame table plane.
    robot_point_1_mm = pixel_to_robot_plane(
        pixel_1_u,
        pixel_1_v,
        GRASP_Z_MM
    )

    robot_point_2_mm = pixel_to_robot_plane(
        pixel_2_u,
        pixel_2_v,
        GRASP_Z_MM
    )

    delta_x_mm = (
        robot_point_2_mm[0]
        - robot_point_1_mm[0]
    )

    delta_y_mm = (
        robot_point_2_mm[1]
        - robot_point_1_mm[1]
    )

    if np.hypot(delta_x_mm, delta_y_mm) < 1e-9:
        raise ValueError(
            "The converted robot-frame direction is too small."
        )

    robot_long_axis_angle_deg = np.degrees(
        np.arctan2(
            delta_y_mm,
            delta_x_mm
        )
    )

    return normalize_parallel_gripper_angle(
        robot_long_axis_angle_deg
    )


def calculate_gripper_j4_deg(
    robot_long_axis_angle_deg,
    current_j1_deg
):
    """
    Calculate J4 so that the gripper closing direction is
    perpendicular to the object's long axis.

    Assumed relationship:

        gripper world direction
        = J1 + J4 + mounting offset
    """

    if GRIPPER_MOUNT_OFFSET_DEG is None:
        raise ValueError(
            "Fill in GRIPPER_MOUNT_OFFSET_DEG first."
        )

    # Gripper closing direction must be perpendicular
    # to the object's long axis.
    target_gripper_world_angle_deg = (
        float(robot_long_axis_angle_deg) + 90.0
    )

    # Solve:
    #
    # target direction
    # = J1 + J4 + mounting offset
    #
    # Therefore:
    #
    # J4 = target direction - J1 - mounting offset
    j4_command_deg = (
        target_gripper_world_angle_deg
        - float(current_j1_deg)
        - float(GRIPPER_MOUNT_OFFSET_DEG)
    )

    return normalize_parallel_gripper_angle(
        j4_command_deg
    )


def get_current_joint_angles(api):
    """
    Return the current J1, J2, J3, and J4 values.

    The expected GetPose output is:

        [x, y, z, r, j1, j2, j3, j4]
    """

    pose = dType.GetPose(api)

    if pose is None or len(pose) < 8:
        raise RuntimeError(
            "Unexpected GetPose() output. "
            "Expected at least 8 values."
        )

    return np.array(
        [
            pose[4],
            pose[5],
            pose[6],
            pose[7]
        ],
        dtype=np.float64
    )


def open_gripper(api):
    """
    Release the suction cup.
    """

    dType.SetEndEffectorSuctionCup(
        api,
        1,
        0,
        0
    )[0]

    dType.dSleep(50)


def close_gripper(api):
    """
    Engage the suction cup.
    """

    dType.SetEndEffectorSuctionCup(
        api,
        1,
        1,
        0
    )[0]

    dType.dSleep(50)


def lab5_motion_parameters_ready(
    require_gripper_angle=False
):
    """
    Check whether the required Lab 5 parameters have been filled in.
    """

    if GRASP_Z_MM is None:
        print("Fill in GRASP_Z_MM first.")
        return False

    if HORIZONTAL_MOVE_Z_MM is None:
        print("Fill in HORIZONTAL_MOVE_Z_MM first.")
        return False

    if (
        float(HORIZONTAL_MOVE_Z_MM)
        <= float(GRASP_Z_MM)
    ):
        print(
            "HORIZONTAL_MOVE_Z_MM must be above GRASP_Z_MM."
        )
        return False

    if (
        require_gripper_angle
        and GRIPPER_MOUNT_OFFSET_DEG is None
    ):
        print(
            "Fill in GRIPPER_MOUNT_OFFSET_DEG first."
        )
        return False

    return True


def apply_gripper_xy_correction(
    robot_position_mm
):
    """
    Apply empirical XY correction to the visually estimated position.
    """

    corrected_position_mm = np.array(
        robot_position_mm,
        dtype=np.float64
    ).copy()

    corrected_position_mm[0] += (
        GRIPPER_X_CORRECTION_MM
    )

    corrected_position_mm[1] += (
        GRIPPER_Y_CORRECTION_MM
    )

    return corrected_position_mm


def move_to_object_hover(
    api,
    robot_position_mm
):
    """
    First rise vertically to the safe movement height, then move
    horizontally to the object's XY position.
    """

    if not lab5_motion_parameters_ready():
        return False

    corrected_position_mm = (
        apply_gripper_xy_correction(
            robot_position_mm
        )
    )

    target_x_mm = float(
        corrected_position_mm[0]
    )

    target_y_mm = float(
        corrected_position_mm[1]
    )

    safe_z_mm = float(
        HORIZONTAL_MOVE_Z_MM
    )

    current_pose = dType.GetPose(api)

    current_x_mm = float(current_pose[0])
    current_y_mm = float(current_pose[1])
    current_z_mm = float(current_pose[2])

    # If the robot is below the horizontal movement height,
    # rise vertically first.
    if current_z_mm < safe_z_mm:
        if not safe_move_to_xyz(
            api,
            current_x_mm,
            current_y_mm,
            safe_z_mm
        ):
            return False

    # Move horizontally at the safe Z height.
    return safe_move_to_xyz(
        api,
        target_x_mm,
        target_y_mm,
        safe_z_mm
    )


def rotate_gripper_for_detection(
    api,
    detection
):
    """
    Rotate J4 so the gripper closing direction is perpendicular
    to the detected object's long axis.
    """

    if not lab5_motion_parameters_ready(
        require_gripper_angle=True
    ):
        return False

    if (
        "robot_long_axis_angle_deg"
        not in detection
    ):
        print(
            "Detection does not contain a "
            "robot-frame long-axis angle."
        )
        return False

    current_joints = get_current_joint_angles(api)

    current_j1_deg = current_joints[0]
    current_j2_deg = current_joints[1]
    current_j3_deg = current_joints[2]

    target_j4_deg = calculate_gripper_j4_deg(
        detection["robot_long_axis_angle_deg"],
        current_j1_deg
    )

    print(
        f"Current J1 = {current_j1_deg:.2f} degrees"
    )

    print(
        f"Target J4 = {target_j4_deg:.2f} degrees"
    )

    move_joint_angles(
        api,
        current_j1_deg,
        current_j2_deg,
        current_j3_deg,
        target_j4_deg
    )

    detection["target_j4_deg"] = (
        target_j4_deg
    )

    return True


def pick_detected_object(
    api,
    detection
):
    """
    Pick one object using:

        safe-height movement
        -> rotate gripper
        -> open gripper
        -> vertical descent
        -> close gripper
        -> vertical lift
    """

    if not lab5_motion_parameters_ready(
        require_gripper_angle=True
    ):
        return False

    if "robot_position_mm" not in detection:
        print(
            "Detection does not contain "
            "a robot-frame position."
        )
        return False

    corrected_position_mm = (
        apply_gripper_xy_correction(
            detection["robot_position_mm"]
        )
    )

    target_x_mm = float(
        corrected_position_mm[0]
    )

    target_y_mm = float(
        corrected_position_mm[1]
    )

    # 1. Move to the object's XY at the safe height.
    if not move_to_object_hover(
        api,
        detection["robot_position_mm"]
    ):
        return False

    # 2. Rotate the gripper.
    if not rotate_gripper_for_detection(
        api,
        detection
    ):
        return False

    # 3. Open the gripper.
    open_gripper(api)

    # 4. Descend vertically.
    if not safe_move_to_xyz(
        api,
        target_x_mm,
        target_y_mm,
        float(GRASP_Z_MM)
    ):
        return False

    # 5. Close the gripper.
    close_gripper(api)

    # 6. Lift vertically.
    if not safe_move_to_xyz(
        api,
        target_x_mm,
        target_y_mm,
        float(HORIZONTAL_MOVE_Z_MM)
    ):
        return False

    return True


def add_robot_data_to_detections(detections):
    """
    Add robot-frame position and long-axis angle
    to every valid object detection.
    """

    processed_detections = []

    if GRASP_Z_MM is None:
        print("Fill in GRASP_Z_MM first.")
        return processed_detections

    for detection in detections:
        pixel_x, pixel_y = detection["pixel_center"]

        try:
            robot_position_mm = pixel_to_robot_plane(
                pixel_x,
                pixel_y,
                GRASP_Z_MM
            )

            robot_long_axis_angle_deg = (
                image_long_axis_to_robot_angle(
                    pixel_x,
                    pixel_y,
                    detection["long_axis_angle_deg"],
                    GRASP_Z_MM
                )
            )

        except ValueError as error:
            print(
                f"Skipping detection at "
                f"{detection['pixel_center']}: "
                f"{error}"
            )
            continue

        detection["robot_position_mm"] = (
            robot_position_mm
        )

        detection["robot_long_axis_angle_deg"] = (
            robot_long_axis_angle_deg
        )

        if HORIZONTAL_MOVE_Z_MM is not None:
            detection["hover_position_mm"] = np.array(
                [
                    robot_position_mm[0],
                    robot_position_mm[1],
                    HORIZONTAL_MOVE_Z_MM
                ],
                dtype=np.float64
            )

        processed_detections.append(detection)

    return processed_detections


def capture_processed_objects():
    """
    Capture one image, detect objects, and add
    robot-frame information to each object.
    """

    frame = capture_camera_frame()

    if frame is None:
        return [], None, None

    detections, debug_image, colour_mask = (
        detect_all_coloured_objects(frame)
    )

    detections = add_robot_data_to_detections(
        detections
    )

    return detections, debug_image, colour_mask


def get_toy_bin_position():
    """
    Detect the selected toy-bin marker and return
    its robot-frame position.
    """

    if TOY_BIN_MARKER_ID is None:
        print("Fill in TOY_BIN_MARKER_ID first.")
        return None

    markers = detect_all_markers_once()

    if TOY_BIN_MARKER_ID not in markers:
        print(
            f"Toy-bin marker "
            f"{TOY_BIN_MARKER_ID} "
            "was not detected."
        )
        return None

    toy_bin_position_mm = np.array(
        markers[TOY_BIN_MARKER_ID]["robot_mm"],
        dtype=np.float64
    ).copy()

    toy_bin_position_mm[0] += (
        TOY_BIN_X_OFFSET_MM
    )

    toy_bin_position_mm[1] += (
        TOY_BIN_Y_OFFSET_MM
    )

    return toy_bin_position_mm


def place_object_in_toy_bin(
    api,
    toy_bin_position_mm
):
    """
    Move a held object to the toy bin,
    release it, and lift away.
    """

    if not lab5_motion_parameters_ready():
        return False

    target_x_mm = float(
        toy_bin_position_mm[0]
    )

    target_y_mm = float(
        toy_bin_position_mm[1]
    )

    safe_z_mm = float(
        HORIZONTAL_MOVE_Z_MM
    )

    current_pose = dType.GetPose(api)

    current_x_mm = float(current_pose[0])
    current_y_mm = float(current_pose[1])
    current_z_mm = float(current_pose[2])

    # Rise vertically first if needed.
    if current_z_mm < safe_z_mm:
        if not safe_move_to_xyz(
            api,
            current_x_mm,
            current_y_mm,
            safe_z_mm
        ):
            return False

    # Move horizontally above the toy bin.
    if not safe_move_to_xyz(
        api,
        target_x_mm,
        target_y_mm,
        safe_z_mm
    ):
        return False

    # Lower the held object.
    if not safe_move_to_xyz(
        api,
        target_x_mm,
        target_y_mm,
        float(GRASP_Z_MM)
    ):
        return False

    # Release the object.
    open_gripper(api)

    # Lift back to the safe height.
    if not safe_move_to_xyz(
        api,
        target_x_mm,
        target_y_mm,
        safe_z_mm
    ):
        return False

    return True


def run_lab5(api):
    """
    Complete Lab 5 M-O task.

    Wait for user input, scan for objects,
    pick one object, move it to the toy bin,
    and repeat.
    """

    if not lab5_motion_parameters_ready(
        require_gripper_angle=True
    ):
        return

    if TOY_BIN_MARKER_ID is None:
        print("Fill in TOY_BIN_MARKER_ID first.")
        return

    initialize_robot(api)

    move_to_home(api)

    print(
        "\nLab 5 is ready. "
        "Keep hands clear whenever "
        "the robot is moving."
    )

    while True:
        command = input(
            "\nPlace or remove objects, "
            "keep your hands clear, "
            "then press Enter to scan. "
            "Enter q to quit: "
        ).strip().lower()

        if command == "q":
            break

        # Move robot out of camera view.
        move_to_home(api)
        time.sleep(1.0)

        # Locate the toy bin.
        toy_bin_position_mm = (
            get_toy_bin_position()
        )

        if toy_bin_position_mm is None:
            print(
                "Could not locate the toy bin. "
                "Try again."
            )
            continue

        # Detect and process objects.
        detections, debug_image, colour_mask = (
            capture_processed_objects()
        )

        if debug_image is not None:
            cv2.imwrite(
                "lab5_latest_detection.png",
                debug_image
            )

        if colour_mask is not None:
            cv2.imwrite(
                "lab5_latest_mask.png",
                colour_mask
            )

        if len(detections) == 0:
            print(
                "No coloured foreign objects "
                "were detected."
            )
            continue

        # Pick one object per scan.
        target_detection = detections[0]

        print(
            "Selected object at robot position "
            f"{target_detection['robot_position_mm'].round(2)} "
            "mm"
        )

        if not pick_detected_object(
            api,
            target_detection
        ):
            print(
                "Object pickup failed "
                "or was rejected."
            )

            move_to_home(api)
            continue

        if not place_object_in_toy_bin(
            api,
            toy_bin_position_mm
        ):
            print(
                "Toy-bin placement failed "
                "or was rejected."
            )

            move_to_home(api)
            continue

        print(
            f"Object moved to toy-bin marker "
            f"{TOY_BIN_MARKER_ID}."
        )

        move_to_home(api)

    move_to_home(api)

    print("Lab 5 stopped.")


def run_all_object_detection_test():
    """
    Open a live camera window and continuously detect coloured objects.

    Press q in the camera window to quit.
    This function does not connect to or move the robot.
    """

    camera_index = 0

    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("Camera could not be opened.")
        return

    # Request the same resolution used by the Lab 5 camera code.
    cap.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG")
    )

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        1920
    )

    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        1080
    )

    print(
        "Live object detection started. "
        "Press q in the camera window to quit."
    )

    while True:
        ret, frame = cap.read()

        if not ret or frame is None:
            print("Could not read a camera frame.")
            break

        detections, debug_image, colour_mask = (
            detect_all_coloured_objects(frame)
        )

        # Add detection count to the displayed image.
        cv2.putText(
            debug_image,
            f"Detected objects: {len(detections)}",
            (30, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 0),
            3
        )

        # Calculate and display robot coordinates when GRASP_Z_MM is set.
        for index, detection in enumerate(detections):
            pixel_x, pixel_y = detection["pixel_center"]

            robot_text = f"Object {index}"

            if GRASP_Z_MM is not None:
                try:
                    robot_position_mm = (
                        pixel_to_robot_plane(
                            pixel_x,
                            pixel_y,
                            GRASP_Z_MM
                        )
                    )

                    robot_long_axis_angle_deg = (
                        image_long_axis_to_robot_angle(
                            pixel_x,
                            pixel_y,
                            detection[
                                "long_axis_angle_deg"
                            ],
                            GRASP_Z_MM
                        )
                    )

                    detection["robot_position_mm"] = (
                        robot_position_mm
                    )

                    detection[
                        "robot_long_axis_angle_deg"
                    ] = robot_long_axis_angle_deg

                    robot_text = (
                        f"Obj {index}: "
                        f"X={robot_position_mm[0]:.1f}, "
                        f"Y={robot_position_mm[1]:.1f}, "
                        f"A={robot_long_axis_angle_deg:.1f}"
                    )

                except ValueError:
                    robot_text = (
                        f"Obj {index}: conversion failed"
                    )

            text_x = max(
                10,
                pixel_x - 120
            )

            text_y = min(
                debug_image.shape[0] - 10,
                pixel_y + 35
            )

            cv2.putText(
                debug_image,
                robot_text,
                (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                2
            )

        # Show the live detection result.
        cv2.imshow(
            "Lab 5 Live Object Detection",
            debug_image
        )

        # Show the live binary mask as a second window.
        cv2.imshow(
            "Lab 5 Colour Mask",
            colour_mask
        )

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            print(
                f"\nDetected {len(detections)} object(s) "
                "in the final camera frame."
            )

            for index, detection in enumerate(detections):
                print(
                    f"\nObject {index}:"
                )

                print(
                    "  Pixel center = "
                    f"{detection['pixel_center']}"
                )

                print(
                    "  Area = "
                    f"{detection['area_px']:.1f} px"
                )

                print(
                    "  Image long-axis angle = "
                    f"{detection['long_axis_angle_deg']:.2f} degrees"
                )

                if "robot_position_mm" in detection:
                    print(
                        "  Robot position = "
                        f"{detection['robot_position_mm'].round(2)} mm"
                    )
                else:
                    print(
                        "  Robot position was not calculated."
                    )

                if "robot_long_axis_angle_deg" in detection:
                    print(
                        "  Robot-frame long-axis angle = "
                        f"{detection['robot_long_axis_angle_deg']:.2f} "
                        "degrees"
                    )
                else:
                    print(
                        "  Robot-frame angle was not calculated."
                    )

            break

    cap.release()
    cv2.destroyAllWindows()

    print("Live object detection stopped.")


################################################################################################
###################################### Lab 5 End ###############################################
################################################################################################



#Before running and commands, always run this
# Before running any commands, always initialize the robot.
# initialize_robot(api)

print("\nSelect a program:")
print("v  - Lab 5 coloured object detection test")
print("l5 - Run complete Lab 5 M-O task")

selection = input(
    "Enter v or l5: "
).strip().lower()

if selection == "v":
    run_all_object_detection_test()

elif selection == "l5":
    run_lab5(api)

else:
    print("Invalid selection.")

# move_to_home(api)
#All done!