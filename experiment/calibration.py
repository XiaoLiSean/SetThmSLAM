# this program is used to calibrate the lidars' pose [x,y,theta]
# using collected measurements from both lidars and optitrack
from lidar import RPLidarA1
from optitrack import OptiTrack
from params import CarToLidar1FoV, DisplacementErrorPercentage
import matplotlib.pyplot as plt
import numpy as np
import copy
# ------------------------------------------------------------------------------
def isInFoV(angle, FoV):
    isIn = False
    for FoV in CarToLidar1FoV['FoVs']:
        if angle > FoV[0] and angle < FoV[1]:
            isIn = True
    return isIn

def filterMeasurement(z):
    filteredMeasurements = dict(r=[],a=[])
    for i in range(len(z['r'])):
        if isInFoV(z['a'][i], CarToLidar1FoV) and z['r'][i] < CarToLidar1FoV['rmax']:
            filteredMeasurements['a'].append(z['a'][i])
            filteredMeasurements['r'].append(z['r'][i])
    filteredMeasurements['r'] = np.array(filteredMeasurements['r'])
    filteredMeasurements['a'] = np.array(filteredMeasurements['a'])
    return filteredMeasurements
# ------------------------------------------------------------------------------
def recordData(filename):
    recording = []
    optitrack = OptiTrack()
    optitrack.startOptiTrackThreaded()
    lidar = RPLidarA1()
    lidar.startLidarThreaded()
    ax = plt.subplot(111)
    while True:
        ax.clear()
        groundTrue = copy.deepcopy(optitrack.position)
        plt.plot(groundTrue[0], groundTrue[1], 'ro', markersize=3)

        measurements = copy.deepcopy(dict(r=np.array(lidar.distances), a=lidar.angles))
        Xs = np.multiply(measurements['r']/1000, np.cos(np.deg2rad(measurements['a'])))
        Ys = np.multiply(measurements['r']/1000, np.sin(np.deg2rad(measurements['a'])))
        plt.plot(Xs, Ys, 'bo', markersize=1)

        markerMeasure = filterMeasurement(measurements)
        MarkerXs = np.multiply(markerMeasure['r']/1000, np.cos(np.deg2rad(markerMeasure['a'])))
        MarkerYs = np.multiply(markerMeasure['r']/1000, np.sin(np.deg2rad(markerMeasure['a'])))
        plt.plot(MarkerXs, MarkerYs, 'rx', markersize=3)

        ax.grid(True)
        plt.pause(0.1)
        datapoint = dict(gt=groundTrue, z=measurements)
        recording.append(copy.deepcopy(datapoint))
        np.save(filename, recording)
    plt.show()
# ------------------------------------------------------------------------------
def replayData(filename):
    recording = np.load(filename, allow_pickle=True)
    ax = plt.subplot(111)
    for data in recording:
        ax.clear()
        groundTrue = data['gt']
        plt.plot(groundTrue[0], groundTrue[1], 'ro', markersize=3)

        measurements = data['z']
        Xs = np.multiply(measurements['r']/1000, np.cos(np.deg2rad(measurements['a'])))
        Ys = np.multiply(measurements['r']/1000, np.sin(np.deg2rad(measurements['a'])))
        plt.plot(Xs, Ys, 'bo', markersize=1)

        markerMeasure = filterMeasurement(measurements)
        MarkerXs = np.multiply(markerMeasure['r']/1000, np.cos(np.deg2rad(markerMeasure['a'])))
        MarkerYs = np.multiply(markerMeasure['r']/1000, np.sin(np.deg2rad(markerMeasure['a'])))
        plt.plot(MarkerXs, MarkerYs, 'rx', markersize=3)

        ax.grid(True)
        ax.axis('equal')
        ax.set_xlim([-5, 5])
        ax.set_ylim([-5, 5])
        plt.pause(0.1)
    plt.show()
# ------------------------------------------------------------------------------
def calibration(filename):
    recording = np.load(filename, allow_pickle=True)
    dataSynchronized = [] # each row is a indivisual recording [gt_x, gt_y, x_measure, y_measure]
    # --------------------------------------------------------------------------
    # Prepare data for calibration calculation
    for data in recording:
        groundTrue = data['gt']
        markerMeasure = filterMeasurement(data['z'])
        # case the lidar or optitrack measurement is empty
        if len(groundTrue)*len(markerMeasure['r']) == 0:
            continue
        # append useful data to dataSynchronized for later calibration
        MarkerXs = np.multiply(markerMeasure['r']/1000, np.cos(np.deg2rad(markerMeasure['a'])))
        MarkerYs = np.multiply(markerMeasure['r']/1000, np.sin(np.deg2rad(markerMeasure['a'])))
        if len(dataSynchronized) == 0:
            if len(MarkerXs) != 1:
                print("First lidar measurement got multiple dots: Xs {}; Ys {}".format(MarkerXs, MarkerYs))
                exit(0)
            dataSynchronized.append([groundTrue[0], groundTrue[1], MarkerXs[0], MarkerYs[0]])
        else:
            # filter out ambiguous lidar measurement
            for i in range(len(MarkerXs)):
                drsq_lidar = (MarkerXs[i]-dataSynchronized[-1][2])**2 + (MarkerYs[i]-dataSynchronized[-1][3])**2
                drsq_opti = (groundTrue[0]-dataSynchronized[-1][2])**2 + (groundTrue[1]-dataSynchronized[-1][3])**2
                if abs(drsq_lidar-drsq_opti)/drsq_opti <= DisplacementErrorPercentage:
                    dataSynchronized.append([groundTrue[0], groundTrue[1], MarkerXs[i], MarkerYs[i]])
    # --------------------------------------------------------------------------
    # Loop through data for visual check
    ax = plt.subplot(111)
    print(dataSynchronized)
    for z in dataSynchronized:
        ax.clear()
        plt.plot(z[0], z[1], 'ro', markersize=3)
        plt.plot(z[2], z[3], 'bx', markersize=1)
        ax.grid(True)
        ax.axis('equal')
        ax.set_xlim([-5, 5])
        ax.set_ylim([-5, 5])
        plt.pause(0.1)
    plt.show()
    # --------------------------------------------------------------------------
    # calibration calculation main

# ------------------------------------------------------------------------------
if __name__ == '__main__':
    filename = 'calibrateRaw.npy'
    # calibration(filename)
    replayData(filename)
    # recordData(filename)
