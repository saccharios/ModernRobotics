import os

import csv
import modern_robotics as mr
import numpy as np

# Define constants
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]


tau=np.array([0,0,0,0,0,0])
g=np.array([0,0,-9.81])
F_tip=np.array([0,0,0,0,0,0])

# Time step for simulation
time_step = 1.0/1000.0


# Run the simulation for seconds with initial configuration theta_init.
# Save every theta of a timestep to file-name
def run(seconds, theta_init, file_name):
    os.remove(file_name)
    current_time = 0
    theta = theta_init
    theta_dot=np.array([0,0,0,0,0,0])
    while current_time < seconds:
        theta_dot_dot = mr.ForwardDynamics(thetalist=theta,dthetalist=theta_dot,taulist=tau,g=g,Ftip=F_tip,Mlist=Mlist,Glist=Glist,Slist=Slist)
        theta, theta_dot = mr.EulerStep(thetalist=theta,dthetalist=theta_dot,ddthetalist=theta_dot_dot,dt=time_step)
        current_time += time_step

        with open(file_name, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerows([theta])


if __name__ == '__main__':
    theta_1=np.array([0, 0, 0, 0, 0, 0])
    run(seconds=3, theta_init=theta_1, file_name='simulation1.csv')

    theta_2=np.array([0, -1, 0, 0, 0, 0])
    run(seconds=5, theta_init=theta_2, file_name='simulation2.csv')

