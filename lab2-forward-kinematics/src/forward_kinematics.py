import DobotDllType as dType
import time
import threading
import numpy as np

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

def run_circular_trajectory(api, num_points=50, num_runs=5):
    """
    Run a circular trajectory inside the workspace.
    Saves trajectory tracking results to CSV.
    """

    center = np.array([200, 0, -10], dtype=np.float64)
    radius = 50

    results = []

    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

    trajectory = []

    # Generate circular trajectory points
    for theta in angles:
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        z = center[2]

        trajectory.append(np.array([x, y, z], dtype=np.float64))

    # Run trajectory multiple times
    for run in range(num_runs):

        print(f"\n--- Trajectory Run {run + 1} ---")

        for i, target in enumerate(trajectory):

            x, y, z = target

            moved = safe_move_to_xyz(api, x, y, z)

            if moved:

                actual_pose = dType.GetPose(api)
                actual_xyz = actual_pose[:3]

                error = np.linalg.norm(actual_xyz - target)

                print(
                    f"Point {i}: "
                    f"target={target.round(1)} "
                    f"actual={actual_xyz.round(1)} "
                    f"error={error:.2f}"
                )

                results.append([
                    run + 1,
                    i,
                    x, y, z,
                    actual_xyz[0],
                    actual_xyz[1],
                    actual_xyz[2],
                    error
                ])

    # Save results
    results = np.array(results)

    header = (
        "run,point,"
        "target_x,target_y,target_z,"
        "actual_x,actual_y,actual_z,"
        "error"
    )

    np.savetxt(
        "part3_trajectory_results.csv",
        results,
        delimiter=",",
        header=header,
        comments=""
    )

    print("\nSaved trajectory results to part3_trajectory_results.csv")

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

    print("\n--- Offline FK Validation ---")
    print(f"Number of points: {len(results)}")
    print(f"Mean error: {np.mean(results[:, 7]):.8f} mm")
    print(f"Max error:  {np.max(results[:, 7]):.8f} mm")
    print(f"Min error:  {np.min(results[:, 7]):.8f} mm")
    print("\n")

    # np.savetxt(
    #     "lab2_offline_fk_validation.csv",
    #     results,
    #     delimiter=",",
    #     header=(
    #         "point,"
    #         "actual_x,actual_y,actual_z,"
    #         "fk_x,fk_y,fk_z,"
    #         "error"
    #     ),
    #     comments=""
    # )

    # print("Saved results to lab2_offline_fk_validation.csv")



# filename = r"C:\Users\bjgeh\Desktop\ECE 486\Lab2\Lab2DesignData.txt"
# validate_offline_data(filename)

###################################### Part 1 End ##############################################


######################################## Part 2 ################################################

def explore_workspace_joint_angles(api):
    """
    Explore workspace boundary points and record
    the corresponding joint angles.
    """

    print("\n--- Part 2 Workspace Exploration ---")

    test_points = [[140, 0, -10],      # inner boundary
                   [260, 0, -10],      # outer boundary
                   [200, 0, 0],        # upper boundary
                   [200, 0, -57],      # Approx lower boundary
                   [10, 230, -10]]      # left boundary
                   #[10, -230, -10]]     # right boundary

    for point in test_points:

        move_to_xyz(api, *point)

        pose = dType.GetPose(api)

        print(f"Point = {point} | "
              f"J1 = {pose[4]:.2f}, "
              f"J2 = {pose[5]:.2f}, "
              f"J3 = {pose[6]:.2f}")

###################################### Part 2 End ##############################################


######################################## Part 3 ################################################

def run_joint_fk_validation(api):
    """
    Validate forward kinematics by moving the robot in joint space,
    reading the actual joint angles and xyz pose, and comparing
    the actual xyz pose with FK-computed xyz.
    """

    print("\n--- Part 3 Joint-Space FK Validation ---")

    test_joints = [[0, 20, 70],
                   [45, 40, 70],
                   [-45, 20, 70],
                   [0, 30, 60],
                   [20, 30, 60],
                   [-20, 30, 60]]

    for joints in test_joints:

        J1, J2, J3 = joints

        if not is_joint_in_workspace(J1, J2, J3):
            print(f"Rejected commanded joints {joints}: outside workspace.")
            continue

        move_joint_angles(api, J1, J2, J3, 0)

        # for step in range(10000):
        #     mj.mj_step(api.env.model, api.env.data)

        pose = dType.GetPose(api)

        actual_xyz = pose[:3]

        actual_J1 = pose[4]
        actual_J2 = pose[5]
        actual_J3 = pose[6]

        fk_xyz = forward_kinematics(actual_J1, actual_J2, actual_J3)

        error = np.linalg.norm(fk_xyz - actual_xyz)

        print(
            f"Commanded joints = {joints} | "
            f"Actual joints = [{actual_J1:.2f}, {actual_J2:.2f}, {actual_J3:.2f}] | "
            f"FK xyz = {fk_xyz.round(2)} | "
            f"Actual xyz = {actual_xyz} | "
            f"Error = {error:.10f} mm"
        )

###################################### Part 3 End ##############################################

################################################################################################
###################################### Lab 2 End ###############################################
################################################################################################




#Before running and commands, always run this
initialize_robot(api)

"""
    Here is a sample script that moves the robot to a position, then moves back to home, then to another position, five times
    
    It also prints the pose of the robot. Then, we move the robot by joint angle just to show how it's done.
"""
# filename = "Lab2DesignData.txt"
# validate_offline_data(filename)

explore_workspace_joint_angles(api)

run_joint_fk_validation(api)

# print("PTP Motions done. Moving in Joint Space now")
# move by joint angles, in degrees    
# move_joint_angles(api,0,45,45)

move_to_home(api)
#All done!