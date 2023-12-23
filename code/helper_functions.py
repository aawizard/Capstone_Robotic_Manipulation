import numpy as np  
import modern_robotics as mr 


def flat(t, g):
    """_Function to format the rows in the required format.
    """
    return np.concatenate([t[0, 0:3], t[1, 0:3], t[2, 0:3], t[0:3, 3], np.array([g])])


def unflat(l):
    t = np.zeros((4, 4))
    t[0, 0:3] = l[0:3]
    t[1, 0:3] = l[3:6]
    t[2, 0:3] = l[6:9]
    t[0:3, 3] = l[9:12]
    t[3, 3] = 1
    g = l[12]

    return t, g

def H(phi):
    ans = []
    l = 0.235
    w = 0.15
    r = 0.0475
    x = [l, l, -l, -l]
    y = [w, -w, -w, w]
    beta = 0
    pi = np.pi/4
    gamma = [-pi, pi, -pi, pi]
    for i in range(4):
        k = (1/(r*np.cos(gamma[i])))
        h = k*np.array([x[i]*np.sin(gamma[i]) - y[i]*np.cos(gamma[i]),
                        np.cos(gamma[i]+phi),
                        np.sin(gamma[i]+phi)
                        ])
        ans.append(h)
    return ans



def get_traj(Xstart, Xend, Tf, N, method, g):
    traj = mr.CartesianTrajectory(Xstart, Xend, Tf, N, method)
    csv = []
    for t in traj:
        k = []
        k = flat(t, g)

        csv.append(k)
    return csv

def get_T_config(config):
    # config is: x,y,phi,j1,j2,j3,j4,j5,w1,w2,w3,w4,g
    # print(config)
    # print(config[0:3])
    phi, x, y = config[0:3]
    thetalist = config[3:8]
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
    T0e = mr.FKinBody(M0e, Blist, thetalist)
    Tsb = np.array([[np.cos(phi), -np.sin(phi), 0, x],
                    [np.sin(phi), np.cos(phi), 0, y],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                    ])
    Tse = Tsb@Tb0@T0e
    return Tse

