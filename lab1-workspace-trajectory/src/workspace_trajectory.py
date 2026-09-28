"""
MuJoCo Simulation Starter Code for DOBOT.
Use this script to run your DOBOT experiments in the simulator.
"""

import argparse
import mujoco as mj
import numpy as np
from dobot_sim_api import (
    SimDobotAPI,
    move_to_xyz,
    move_joint_angles,
    get_pose,
    engage_suction,
    release_suction,
    stop_pump,
    move_to_home,
    HOME_CTRL
)
from dobot_mujoco.env.dobot_pick_place import DobotPickPlace

# Pick and Place joint targets (measured from demo)
PICK_JOINTS = np.array([-24.3, 54.9, 39.9, 73.7], dtype=np.float64)
PLACE_LIFT_JOINTS = np.array([33.2, 30.9, 25.2, 40.1], dtype=np.float64)
PLACE_JOINTS = np.array([39.4, 43.2, 47.7, 38.9], dtype=np.float64)


def create_sim_api(seed: int, headless: bool) -> SimDobotAPI:
    """Factory to create the simulator environment and API object."""
    env = DobotPickPlace(render_mode=None, position_jitter=0.0)
    env.reset(seed=seed)

    viewer = None
    if not headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(env.model, env.data)

    dobot_body_id = env.model.body("dobot").id
    base_pos_mm = env.data.xpos[dobot_body_id].copy() * 1000.0

    api = SimDobotAPI(
        env=env,
        viewer=viewer,
        home_pos=np.zeros(3, dtype=np.float64),
        base_pos_mm=base_pos_mm,
    )
    api.home_pos = api.current_xyz_mm()
    return api


def initialize_robot(api: SimDobotAPI) -> None:
    """Initialize robot state and drive to home position."""
    api.suction_on = False
    api.env.suction_activated = False
    api.env.data.ctrl[:4] = HOME_CTRL
    api.env.data.ctrl[4] = 0.0
    for step in range(250):
        mj.mj_step(api.env.model, api.env.data)
        api.sync_viewer(every=1, step=step)
    api.home_pos = api.current_xyz_mm()
    print(f"Simulator ready. home_pos = {api.home_pos.round(1).tolist()} mm")


def print_status(api: SimDobotAPI, label: str) -> None:
    """Print current robot and task status."""
    obs = api.env._get_obs()
    info = api.env._get_info(obs)
    cube_pos = api.env.data.body("pick_cube").xpos.copy() * 1000.0
    pose = get_pose(api)
    print(
        f"{label}: xyz_mm={pose[:3].round(1)} joints_deg={pose[4:].round(1)} "
        f"suction={api.suction_on} grasped={info['grasped']} success={info['is_success']} "
        f"cube_to_goal={info['cube_to_goal_distance']:.4f} cube_mm={cube_pos.round(1)}"
    )




################################################################################################
######################################## Part 2 ################################################
################################################################################################

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
    current_pose = get_pose(api)
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
###################################### Part 2 End ##############################################
################################################################################################




################################################################################################
######################################## Part 3 ################################################
################################################################################################

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

                actual_pose = get_pose(api)
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

################################################################################################
###################################### Part 3 End ##############################################
################################################################################################



def main() -> None:
    parser = argparse.ArgumentParser(description="DOBOT Simulation Starter Script.")
    parser.add_argument("--seed", type=int, default=0, help="Environment seed.")
    parser.add_argument("--headless", action="store_true", help="Run without the MuJoCo viewer.")
    args = parser.parse_args()

    api = create_sim_api(seed=args.seed, headless=args.headless)

    try:
        initialize_robot(api)
        print_status(api, "home")

        # Example 1: Move using Cartesian Coordinates (Analytical IK)
        print("\n--- Cartesian Motion Demo ---")

        # Home is outside the restricted workspace, so first enter the workspace directly.
        move_to_xyz(api, 200, 0, -50)
        print_status(api, "entered_workspace")

        # Valid move: target and path should both stay inside the workspace.
        safe_move_to_xyz(api, 180, 150, -80)
        print_status(api, "valid_target")

        # Invalid: r < 140.
        safe_move_to_xyz(api, 100, 0, -50)
        print_status(api, "invalid_small_radius")

        # Invalid: r > 260.
        safe_move_to_xyz(api, 270, 0, -50)
        print_status(api, "invalid_large_radius")

        # Invalid: z < -120.
        safe_move_to_xyz(api, 200, 0, -130)
        print_status(api, "invalid_z")


        #safe_move_to_xyz(api, 140, 0, -10)    # inner boundary
        #safe_move_to_xyz(api, 260, 0, -10)    # outer boundary
        #safe_move_to_xyz(api, 200, 0, 0)      # upper boundary
        #safe_move_to_xyz(api, 200, 0, -120)   # lower boundary


        # Circular trajectory
        move_to_xyz(api, 200, 0, -10)
        print_status(api, "reset_before_trajectory")

        run_circular_trajectory(api)

        # Example 2: Pick and Place sequence using Joint Angles
        #print("\n--- Pick and Place Demo ---")
        #move_to_home(api)
        
        #move_joint_angles(api, *PICK_JOINTS)
        #engage_suction(api)
        #print_status(api, "grasped_cube")

        #move_joint_angles(api, *PLACE_LIFT_JOINTS)
        #move_joint_angles(api, *PLACE_JOINTS)
        #release_suction(api)
        #print_status(api, "released_cube")

        #move_to_home(api)
        #print_status(api, "back_home")
        
    finally:
        stop_pump(api)
        if api.viewer is not None:
            api.viewer.close()
        api.env.close()


if __name__ == "__main__":
    main()
