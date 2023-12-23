from helper_functions import H
import numpy as np


def NextState(curr_config, control, dt, max_w):
    """simulator for the kinematics of the youBot

    Args:
        curr_config : current configuration of the robot
        control : 9-vector of controls indicating the wheel speeds and joint velocities
        dt : timestamp
        max_w : maximaun angular velocity

    """

    F = np.linalg.pinv(H(curr_config[0]))

    chassie_state = curr_config[:3]    # size : 3 , x,y,phi
    arm_state = curr_config[3:8]       # size : 5
    wheel_angle_state = curr_config[8:12]  # size : 4
    g = curr_config[12]
    wheel_control = np.array([min(max_w, max(-max_w, num))
                              for num in control[:4]])  # size : 4
    joint_speed_control = np.array([min(max_w, max(-max_w, num))
                                    for num in control[4:]])  # size : 5

    qb = F@(wheel_control)*dt

    if abs(qb[0]) < 0.00001:
        qb = np.array([0, qb[1], qb[2]])
    else:
        w, vx, vy = qb
        qb = np.array([w,
                       (vx*np.sin(w)+vy*(np.cos(w)-1))/w,
                       (vy*np.sin(w)+vx*(1-np.cos(w)))/w,

                       ])

    new_arm_joints = arm_state + joint_speed_control*dt
    new_wheel_angles = wheel_angle_state + wheel_control*dt
    new_chassis_state = chassie_state + qb

    new_state = np.concatenate(
        (new_chassis_state, new_arm_joints, new_wheel_angles, [g]))
    return new_state


def test_milestone1():
    u = np.array([10.0, 10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    dt = 0.01
    pi = np.pi/3
    curr = np.array([0.0, 0.0, 0.0, pi, pi, pi, pi, pi, 0.0, 0.0, 0.0, 0.0, 0])
    csv = [curr]
    for i in range(200):

        curr = NextState(curr_config=curr, dt=dt, max_w=5, control=u)
        csv.append(curr)
    np.savetxt('milestonr1.csv', csv, delimiter=',')
