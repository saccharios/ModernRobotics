import pip


def install():
    if hasattr(pip, 'main'):
        pip.main(['install', 'modern_robotics'])
    else:
        pip._internal.main(['install', 'modern_robotics'])


import modern_robotics as mr

import numpy as np

if __name__ == '__main__':
    # install()
    x = mr.QuinticTimeScaling(Tf=5,t=3)
    print(f"x = {x}")


    Xstart = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
    Xend = np.array([[0, 0, 1, 1],
                     [1, 0, 0,   2],
                     [0, 1, 0, 3],
                     [0, 0, 0,   1]])

    st = mr.ScrewTrajectory(Xstart=Xstart,Xend=Xend,Tf=10,N=10,method=3)
    print(f"st = {st[8]}")
    ct = mr.CartesianTrajectory(Xstart=Xstart,Xend=Xend,Tf=10,N=10,method=5)
    print(f"ct = {ct[8]}")