# this program is used to calibrate the lidars' pose [x,y,theta]
# using collected measurements from both lidars and optitrack
from lidar import RPLidarA1
from optitrack import OptiTrack
from params import CarToLidar1FoV
import matplotlib.pyplot as plt
import numpy as np
import copy

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
        plt.pause(0.1)
    plt.show()

def calibration(filename):
    pass

if __name__ == '__main__':
    filename = 'calibrateRaw.npy'
    replayData(filename)
    #recordData(filename)
