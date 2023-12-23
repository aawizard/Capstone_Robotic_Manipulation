import modern_robotics as mr


def FeedbackControl(X, X_d, X_d_next, Kp, Ki, dt, integrate_term):
    """calculate the kinematic task-space feedforward plus feedback control law

                Input:
                :param X: The current actual end-effector configuration
                :param X_d: The current end-effector reference configuration
                :param X_d_next: The end-effector reference configuration at the next timestep in the reference trajectory
                :param Kp: the P gain matrix
                :param Ki: the I gain matrix
                :param time_step: A timestep delta t between reference trajectory configurations


                Output: 
                :param V: The commanded end-effector twist expressed in the end-effector frame
                :a list of 13 entries csv file (the 12-vector consisting of 3 chassis configuration variables, 
                the 5 arm joint angles, and the 4 wheel angles, plus a "0" for "gripper open") representing the robot's 
                configuration after each integration step

                Other param:
                :param thetalist: A list of joint coordinates
                :param Blist: The joint screw axes in the end-effector frame when the
                                manipulator is at the home position
        speed_max = 12
        """

    X_d, g = X_d
    X_d_next = X_d_next[0]

    Xerr = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(X)@X_d))
    Vd = mr.se3ToVec((1/dt)*mr.MatrixLog6(mr.TransInv(X_d)@X_d_next))

    term1 = mr.Adjoint(mr.TransInv(X)@X_d)@Vd

    term2 = Kp*Xerr
    term3 = Ki*Xerr*dt + integrate_term
    Vt = term1 + term2

    return Vt, term3, Xerr

