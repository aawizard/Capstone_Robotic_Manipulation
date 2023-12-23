import numpy as np
from helper_functions import get_traj


def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, k):
    """_summary_

    Args:
        Tse_init: The initial configuration of the end-effector in the reference trajectory
        Tsc_init : The cube's initial configuration
        Tsc_final : The cube's desired final configuration
        Tce_grasp : The end-effector's configuration relative to the cube when it is grasping the cube
        Tce_standoff : The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
        k : The number of trajectory reference configurations per 0.01 seconds
    """
    # part1 of trajectory:

    start = Tse_init
    end = Tsc_init @ Tce_standoff
    Tf = 9
    N = k*100
    method = 3
    csv = get_traj(start, end, Tf, N, method, 0)

    # part2: going to the block
    start = end
    end = Tsc_init @ Tce_grasp
    Tf = 2.5
    csv.extend(get_traj(start, end, Tf, N, method, 0))

    # part3
    start = end
    end = end
    Tf = 1
    csv.extend(get_traj(start, end, Tf, N, method, 1))

    # part4: taking to gripper back up
    start = end
    end = Tsc_init @ Tce_standoff
    Tf = 2.5
    csv.extend(get_traj(start, end, Tf, N, method, 1))

    # part5: to final standoff
    start = end
    end = Tsc_final @ Tce_standoff
    Tf = 9
    csv.extend(get_traj(start, end, Tf, N, method, 1))

    # part6: placing it down
    start = end
    end = Tsc_final @ Tce_grasp
    Tf = 2.5
    csv.extend(get_traj(start, end, Tf, N, method, 1))

    # part7
    start = end
    end = end
    Tf = 1
    csv.extend(get_traj(start, end, Tf, N, method, 0))

    # part8: taking the hand back up
    start = end
    end = Tsc_final @ Tce_standoff
    Tf = 2.5
    csv.extend(get_traj(start, end, Tf, N, method, 0))

    np.savetxt('trajectory.csv', csv, delimiter=',')
    return csv


def test_milestone2():

    Tsc_init = np.array([[1, 0, 0, 1],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0.025],
                        [0, 0, 0, 1]])

    Tsc_final = np.array([[0, 1, 0, 0],
                          [-1, 0, 0, -1],
                          [0, 0, 1, 0.025],
                          [0, 0, 0, 1]])

    Tse_init = np.array([[0, 0, 1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0, 1]])

    Tce_standoff = np.array([[-0.707, 0, 0.707, 0],
                            [0, 1, 0, 0],
                            [-0.707, 0, -0.707, 0.25],
                            [0, 0, 0, 1]])

    Tce_grasp = np.array([[-0.707, 0, 0.707, 0],
                          [0, 1, 0, 0],
                          [-0.707, 0, -0.707, 0],
                          [0, 0, 0, 1]])
    TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final,
                        Tce_grasp, Tce_standoff, 1)
