import pip

import modern_robotics as mr
import numpy as np

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """
    thetalist = np.array(thetalist0).copy()
    theta_iterations = np.matrix(thetalist)
    i = 0
    maxiterations = 20
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    while err and i < maxiterations:
        thetalist = thetalist  + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
        # Add a row to the theta_iterations
        theta_iterations = np.concatenate((theta_iterations, [thetalist]), axis=0)
        # Save f_kin_body for printing
        f_kin_body = mr.FKinBody(M, Blist, thetalist)
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(f_kin_body), T)))

        err_omega = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        err_linear = np.linalg.norm([Vb[3], Vb[4], Vb[5]])

        i = i + 1
        print_statistics(num_iteration=i,joint_vector=thetalist, end_effector_config=f_kin_body,Vb=Vb,err_omega=err_omega, err_linear=err_linear)
        err = err_omega > eomg or err_linear > ev

    return (thetalist, not err, theta_iterations)


def print_statistics(num_iteration, joint_vector,end_effector_config, Vb, err_omega, err_linear):
    digits = 2
    print(f"""
Iteration {num_iteration}:
joint_vector: {joint_vector.round(digits)}
end_effector_config: {end_effector_config.round(digits)}
Vb: {Vb.round(digits)}
err_omega: {err_omega}
err_linear: {err_linear}
""")


if __name__ == '__main__':
    W1 = 0.109
    W2 = 0.082
    L1 = 0.425
    L2 = 0.392
    H1 = 0.089
    H2 = 0.095

    Blist = np.array([[0, 1, 0, W1+W2, 0,   L1+L2],
                      [0, 0,  1, H2, -L1-L2,   0],
                      [0, 0,  1, H2, -L1, 0],
                      [0, 0,  1, H2, 0, 0],
                      [0, -1,  0, -W2, 0, 0],
                      [0, 0,  1, 0, 0, 0]
                      ]).T
    M = np.array([[-1, 0,  0, L1+L2],
                  [ 0, 0,  1, W1+W2],
                  [ 0, 1, 0, H1-H2],
                  [ 0, 0,  0, 1]])
    T = np.array([[0, 1,  0,     -0.5],
                  [0, 0,  -1,      0.1],
                  [-1, 0, 0,0.1],
                  [0, 0,  0,      1]])
    thetalist0 = np.array([2,0.5,-1,1,-0.5,-1.5])
    eomg = 0.001
    ev = 0.0001




    theta_solution, no_error,theta_iterations = IKinBodyIterates(Blist, M, T, thetalist0, eomg,ev)
    print(f"theta_solution = {theta_solution}")
    print(f"no error = {no_error}")
    print(f"theta_iterations = {theta_iterations}")