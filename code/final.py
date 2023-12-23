from milestone1 import NextState
from milestone2 import TrajectoryGenerator
from milestone3 import FeedbackControl
from helper_functions import get_T_config, unflat, H
import numpy as np 
import matplotlib.pyplot as plt
import modern_robotics as mr


def Controller():

    Tsc_init = np.array([[1, 0, 0, 1.00],
                        [0, 1, 0, 1.00],
                        [0, 0, 1, -0.075],
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

    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0,   0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,   1]])

    M0e = np.array([[1, 0, 0, 0.033],
                    [0, 1, 0,   0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0,   1]])

    Blist = np.array([[0, 0, 1,   0, 0.033, 0],
                      [0, -1, 0, -0.5076,  0, 0],
                      [0, -1, 0, -0.3526,  0, 0],
                      [0, -1, 0, -0.2176,  0, 0],
                      [0, 0, 1,   0,     0, 0]]).T

    init_config = np.array(
        [0.0,	0.2,	0.0,	0.0,	0.0,	0.0,	-1.4,	0.0,	0.0,	0.0,	0.0,	0.0,	0,])

    config = init_config
    config_list = [config]
    traj_list = TrajectoryGenerator(
        Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_standoff, 1)
    print("Trajectory generated")
    X = Tse_init
    Kp = 2.5
    Ki = 0.001
    F = np.linalg.pinv(H(0))
    F6 = np.zeros(shape=(6, 4))
    integrate_term = 0
    F6[2:-1, :] = F
    X_err_list = []
    # print(F)
    print("Generating Anumation csv file")
    for i in range(len(traj_list)-1):
        X_d = unflat(traj_list[i])
        X_d_next = unflat(traj_list[i+1])
        # Calling FeedbackControl that will give the twist in eneffector frame
        V, integrate_term, Xerr = FeedbackControl(
            X, X_d, X_d_next, Kp, Ki, 0.01, integrate_term)
        X_err_list.append(Xerr)
        theta_list = config[3:8]
        Ja = mr.JacobianBody(Blist, theta_list)  # jacobian for arm angles
        T0e = mr.FKinBody(M0e, Blist, theta_list)

        Jb = mr.Adjoint(np.dot(mr.TransInv(T0e), mr.TransInv(Tb0)))@F6
        Je = np.concatenate((Jb, Ja), axis=1)
        Je_inv = np.linalg.pinv(Je)
        control = Je_inv @ V
        config = NextState(config, control, 0.01, 100)
        config[12] = traj_list[i+1][12]
        # phi,x,y,j1,j2,j3,j4,j5,w1,w2,w3,w4,g
        config_list.append(config)
        # print(len(config))
        X = get_T_config(config)
    np.savetxt('ans.csv', config_list, delimiter=',')
    print("Animation csv file generaated")
    plt.plot(list(range(len(X_err_list))), X_err_list)
    plt.show()


Controller()