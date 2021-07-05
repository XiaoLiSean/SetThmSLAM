# this program is used to calibrate the lidars' pose [x,y,theta]
# using collected measurements from both lidars and optitrack
from sensors.lidar import RPLidarA1
from sensors.optitrack import OptiTrack
from scipy.optimize import curve_fit
from params import CarToLidar1FoV, PosErrorMax
import matplotlib.pyplot as plt
import numpy as np
import copy, time
# ==============================================================================
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
# ==============================================================================
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
        plt.pause(0.01)
        datapoint = dict(t=time.time(), gt=groundTrue, z=measurements)
        recording.append(copy.deepcopy(datapoint))
        np.save(filename, recording)
    plt.show()
# ==============================================================================
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
        plt.pause(0.01)
    plt.show()
# ==============================================================================
def solveLidarPose(dataSynchronized):
    # calibration calculation main: variables are [cos(t), sin(t), dx, dy]
    # where the lidar measurement (x2, y2) and global pose (x1, y1) by OptiTrack
    # can be formulated as:
    # x1 = x2*cos(t) - y2*sin(t) + dx;
    # y1 = x2*sin(t) + y2*cos(t) + dy;
    A = []
    b = []
    for data in dataSynchronized:
        x1 = data[1]
        y1 = data[2]
        x2 = data[3]
        y2 = data[4]
        A.append([x2, -y2, 1, 0])
        b.append([x1])
        A.append([y2, x2, 0, 1])
        b.append([y1])
    # least square optimization where the cosine and sine constraint is neglected
    lidar_pose = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)[0]
    theta = np.arctan2(lidar_pose[1], lidar_pose[0])
    dx = lidar_pose[2]
    dy = lidar_pose[3]
    return dx, dy, theta
# ------------------------------------------------------------------------------
def obtainMaximumBound(dataSynchronized, dx, dy, theta):
    e_a = 0
    e_r = 0
    v_max = 0
    for idx, z in enumerate(dataSynchronized):
        x_gt = z[1]
        y_gt = z[2]
        bearing_z = z[5]
        range_z = z[6]
        bearing_gt = np.arctan2(y_gt-dy, x_gt-dx) - theta
        range_gt = np.sqrt((y_gt-dy)**2+(x_gt-dx)**2)
        if abs(bearing_gt-bearing_z) > e_a:
            e_a = abs(bearing_gt-bearing_z)
        if abs(range_gt-range_z) > e_r:
            e_r = abs(range_gt-range_z)
        if idx > 0:
            x_gt_prev = dataSynchronized[idx-1][1]
            y_gt_prev = dataSynchronized[idx-1][2]
            displacement = np.sqrt((x_gt_prev-x_gt)**2+(y_gt_prev-y_gt)**2)
            dt = z[0] - dataSynchronized[idx-1][0]
            speed = displacement / dt
            if speed < 0:
                print("Get Negative speed with dt = {} [sec]".format(dt))
                exit()
            if speed > v_max:
                v_max = speed
    return e_a, e_r, v_max
# ------------------------------------------------------------------------------
def saveDataAndCalibrationParam(dataSynchronized, dx, dy, theta, e_a, e_r, v_max):
    calibratedData = []
    for z in dataSynchronized:
        calibratedData.append([z[0],z[1],z[2],z[5],z[6]])
    calibratedData = np.array(calibratedData)
    calibrationParams = np.array([dx, dy, theta, e_a, e_r, v_max]).reshape((1,6))
    np.savetxt('calibrationData/calibratedData.txt', calibratedData, delimiter=',')
    np.savetxt('calibrationData/calibrationParams.txt', calibrationParams, delimiter=',')
# ------------------------------------------------------------------------------
def calibration(filename):
    recording = np.load(filename, allow_pickle=True)
    dataSynchronized = [] # each row is a indivisual recording [timestamp-t0, gt_x, gt_y, x_measure, y_measure, bearing, range]
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
            if len(dataSynchronized) == 0:
                t0 = data['t']
            else:
                t0 = dataSynchronized[0][0]
            dataSynchronized.append([data['t']-t0, groundTrue[0], groundTrue[1], MarkerXs[i], MarkerYs[i], np.deg2rad(markerMeasure['a'])[i], markerMeasure['r'][i]/1000])
    # --------------------------------------------------------------------------
    # solve/resolve lidar pose use un-filtered/filtered dataSynchronized
    dx, dy, theta = solveLidarPose(dataSynchronized)
    tmp_dataSynchronized = copy.deepcopy(dataSynchronized)
    for z in tmp_dataSynchronized:
        pos_opti = np.array([z[1], z[2]]).reshape((2,1))
        pos_lidar = np.array([z[3]*np.cos(theta)-z[4]*np.sin(theta)+dx, z[3]*np.sin(theta)+z[4]*np.cos(theta)+dy]).reshape((2,1))
        l2error = np.linalg.norm(pos_opti-pos_lidar, ord=2)
        if l2error > PosErrorMax:
            dataSynchronized.remove(z)
    dx, dy, theta = solveLidarPose(dataSynchronized)
    e_a, e_r, v_max = obtainMaximumBound(dataSynchronized, dx, dy, theta)
    saveDataAndCalibrationParam(dataSynchronized, dx, dy, theta, e_a, e_r, v_max)
    print('-----'*20)
    print("Calibration result: [dx, dy, theta] = ({}[m],{}[m],{}[deg])".format(dx, dy, np.rad2deg(theta)))
    print("Maximum error/speed bound: bearing {}[deg]; range {}[m]; speed {}[m/s]".format(np.rad2deg(e_a), e_r, v_max))
    print('-----'*20)
    # --------------------------------------------------------------------------
    # Loop through data for visual check
    ax = plt.subplot(111)
    for z in dataSynchronized:
        ax.clear()
        plt.plot(z[1], z[2], 'ro', markersize=3)
        plt.plot(z[3]*np.cos(theta)-z[4]*np.sin(theta)+dx, z[3]*np.sin(theta)+z[4]*np.cos(theta)+dy, 'bx', markersize=3)
        ax.grid(True)
        ax.axis('equal')
        ax.set_xlim([-5, 5])
        ax.set_ylim([-5, 5])
        plt.pause(0.01)
    plt.show()

# ==============================================================================
if __name__ == '__main__':
    filename = 'calibrationData/calibrateRaw.npy'
    # recordData(filename)
    # replayData(filename)
    calibration(filename)