import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("please input valida file!")
        exit(-1)
    path = sys.argv[1]
    path_data = np.loadtxt(path)
    plt.rcParams["figure.figsize"] = (16.0,12.0)

    # trajectory
    plt.subplot(121)
    plt.scatter(path_data[:,1], path_data[:,2], s=2)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid()
    plt.title("2D trajectory")

    # 姿态
    plt.subplot(222)
    plt.plot(path_data[:,0], path_data[:,4], 'r')
    plt.plot(path_data[:,0], path_data[:,5], 'g')
    plt.plot(path_data[:,0], path_data[:,6], 'b')
    plt.plot(path_data[:,0], path_data[:,7], 'k')
    plt.title('q')

    # 速度
    plt.subplot(224)
    plt.plot(path_data[:,0], path_data[:,8], 'r')
    plt.plot(path_data[:,0], path_data[:,9], 'g')
    plt.plot(path_data[:,0], path_data[:,10], 'b')
    plt.title('v')
    plt.legend(['vx', 'vy', 'vz'])

    plt.show()
    exit(1)
