import matplotlib.pyplot as plt
from lidar import RPLidarA1
import numpy as np
import time, threading, serial
import matplotlib.animation as animation

PORT_NAME = 'COM5'

# ------------------------------------------------------------------------------
def rplidar_thread(lidar):
    lidar.update()
    lidar.shutdown()

def calculate_distance_at_zero_deg(angles, distances):
    idx = np.where(np.abs(angles) <= np.deg2rad(3))[0]
    range = distances[np.abs(angles) <= np.deg2rad(3)]
    print('Range: {} mm'.format(np.mean(range)))
    return

# ------------------------------------------------------------------------------
if __name__ == '__main__':
    # run()
    print('sds')
    lidar = RPLidarA1(PORT_NAME)
    t1 = threading.Thread(target=rplidar_thread, args=(lidar, ))
    t1.start()

    ax = plt.subplot(111, projection='polar')

    while True:
        ax.clear()
        angles = np.array(lidar.angles)*np.pi/180
        distances = np.array(lidar.distances)
        plt.polar(angles, distances, 'ro', markersize=2)
        calculate_distance_at_zero_deg(angles, distances)
        ax.set_rmax(3000)
        ax.grid(True)
        plt.pause(0.1)

    plt.show()
    t1.join()
