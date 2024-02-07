import numpy as np
import matplotlib.pyplot as plt

ekf_x = np.load('logs_ekf_x.npy')
x = np.load('logs_x.npy')
xdr = np.load('logs_xdr.npy')
dock = np.load('logs_dock.npy')

plt.axis('equal')
for i in range(len(ekf_x[0, 1:])):
    plt.cla()
    plt.scatter(x[0, i+1], x[1, i+1], color='blue')
    plt.scatter(ekf_x[0, i+1], ekf_x[1, i+1], color='red')
    plt.scatter(xdr[0, i + 1], xdr[1, i + 1], color='black')
    plt.plot(x[0, 1:], x[1, 1:], color='blue', label='x')
    plt.plot(xdr[0, 1:], xdr[1, 1:], color='black', label='xdr')
    plt.plot(ekf_x[0, 1:], ekf_x[1, 1:], color='red', label='ekf_x')
    plt.scatter(dock[0, 1], dock[1, 1], label='dock')
    # plt.legend()

    plt.pause(.016)

plt.show()