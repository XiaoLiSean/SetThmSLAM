# this program is used to calibrate the lidars' pose [x,y,theta]
# using collected measurements from both lidars and optitrack
from lidar import RPLidarA1
from optitrack import OptiTrack
from scipy.optimize import curve_fit
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
def measurementFcn(x, dx, dy, theta):
    print(x[0].shape)
    return x[0]*np.cos(theta) + x[1]*np.sin(theta) + x[2]*dx + x[3]*dy

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
        for i in range(len(MarkerXs)):
            dataSynchronized.append([groundTrue[0], groundTrue[1], MarkerXs[i], MarkerYs[i]])
    # --------------------------------------------------------------------------
    # calibration calculation main: variables are [cos(t), sin(t), dx, dy]
    # where the lidar measurement (x2, y2) and global pose (x1, y1) by OptiTrack
    # can be formulated as:
    # x1 = x2*cos(t) - y2*sin(t) + dx;
    # y1 = x2*sin(t) + y2*cos(t) + dy;
    A = []
    b = []
    for data in dataSynchronized:
        x1 = data[0]
        y1 = data[1]
        x2 = data[2]
        y2 = data[3]
        A.append([x2, -y2, 1, 0])
        b.append([x1])
        A.append([y2, x2, 0, 1])
        b.append([y1])
    # least square optimization where the cosine and sine constraint is neglected
    lidar_pose = np.linalg.lstsq(A, b)[0]
    theta = np.arctan2(lidar_pose[1], lidar_pose[0])
    dx = lidar_pose[2]
    dy = lidar_pose[3]
    # uncomment for nonlinear optimization
    # Xdata = np.array(A).reshape((4,len(b)))
    # Ydata = np.array(b).reshape((len(b),))
    # popt, pcov = curve_fit(measurementFcn, Xdata, Ydata)
    # dx = popt[0]
    # dy = popt[1]
    # theta = popt[2]
    print("Calibration result: [dx, dy, theta]=({}[m],{}[m],{}[deg])".format(dx, dy, np.rad2deg(theta)))
    # --------------------------------------------------------------------------
    # Loop through data for visual check
    ax = plt.subplot(111)
    for z in dataSynchronized:
        ax.clear()
        plt.plot(z[0], z[1], 'ro', markersize=3)
        plt.plot(z[2]*np.cos(theta)-z[3]*np.sin(theta)+dx, z[2]*np.sin(theta)+z[3]*np.cos(theta)+dy, 'bx', markersize=1)
        ax.grid(True)
        ax.axis('equal')
        ax.set_xlim([-5, 5])
        ax.set_ylim([-5, 5])
        plt.pause(0.01)
    plt.show()

# ------------------------------------------------------------------------------
if __name__ == '__main__':
    filename = 'calibrateRaw.npy'
    # recordData(filename)
    # replayData(filename)
    calibration(filename)
