# ==============================================================================
# this program is used to calibrate the lidars' pose [x,y,theta]
# using collected measurements from both lidars and optitrack
# ==============================================================================
'''Important Note!!!: as the new RPLidarA1M8R6 has different usb adaptor'
please first connect RPLidarA1M8R5 through usb wire to the PC which can activate
the dev/ttyUSB0 port. Afterwards (only by this means), RPLidarA1M8R6 connecting
to the PC can activate ttyUSB1 and ttyUSB2'''
# ==============================================================================
from sensors.lidar import RPLidarA1
from sensors.optitrack import OptiTrack
from scipy.optimize import curve_fit
from params import LIDAR_PORTs, CarToLidarFoVs, DistanceThreshold
import matplotlib.pyplot as plt
import numpy as np
import copy, time, csv
# ==============================================================================
def isInFoV(angle, CarToLidarFoV):
    isIn = False
    for FoV in CarToLidarFoV['FoVs']:
        if angle > FoV[0] and angle < FoV[1]:
            isIn = True
    return isIn

def filterMeasurement(z, CarToLidarFoV):
    filteredMeasurements = dict(r=[],a=[])
    for i in range(len(z['r'])):
        if isInFoV(z['a'][i], CarToLidarFoV) and z['r'][i] < CarToLidarFoV['rmax']:
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
    lidars = []
    for lidar_port in LIDAR_PORTs:
        lidars.append(RPLidarA1(lidar_port))
        lidars[-1].startLidarThreaded()

    fig, axs = plt.subplots(ncols=len(LIDAR_PORTs))
    while True:
        groundTrue = copy.deepcopy(optitrack.position)
        measurements = []
        for idx,lidar in enumerate(lidars):
            axs[idx].clear()

            axs[idx].plot(groundTrue[0], groundTrue[1], 'ro', markersize=3)

            measurement = dict(r=np.array(lidar.distances), a=lidar.angles)
            measurements.append(copy.deepcopy(measurement))
            Xs = np.multiply(measurement['r']/1000, np.cos(np.deg2rad(measurement['a'])))
            Ys = np.multiply(measurement['r']/1000, np.sin(np.deg2rad(measurement['a'])))
            axs[idx].plot(Xs, Ys, 'bo', markersize=1)

            markerMeasure = filterMeasurement(measurement, CarToLidarFoVs[idx])
            MarkerXs = np.multiply(markerMeasure['r']/1000, np.cos(np.deg2rad(markerMeasure['a'])))
            MarkerYs = np.multiply(markerMeasure['r']/1000, np.sin(np.deg2rad(markerMeasure['a'])))
            axs[idx].plot(MarkerXs, MarkerYs, 'rx', markersize=3)
            axs[idx].set_xlim([-2, 2])
            axs[idx].set_ylim([-2, 2])
            axs[idx].grid(True)
            axs[idx].axis('equal')

        plt.pause(0.01)
        datapoint = dict(t=time.time(), gt=groundTrue, z=measurements)
        recording.append(copy.deepcopy(datapoint))
        np.save(filename, recording)
    plt.show()
# ==============================================================================
def replayData(filename):
    recording = np.load(filename, allow_pickle=True)
    fig, axs = plt.subplots(ncols=len(LIDAR_PORTs))
    for data in recording:
        groundTrue = data['gt']
        for idx in range(len(LIDAR_PORTs)):
            axs[idx].clear()

            axs[idx].plot(groundTrue[0], groundTrue[1], 'ro', markersize=3)

            measurement = data['z'][idx]
            Xs = np.multiply(measurement['r']/1000, np.cos(np.deg2rad(measurement['a'])))
            Ys = np.multiply(measurement['r']/1000, np.sin(np.deg2rad(measurement['a'])))
            axs[idx].plot(Xs, Ys, 'bo', markersize=1)

            markerMeasure = filterMeasurement(measurement, CarToLidarFoVs[idx])
            MarkerXs = np.multiply(markerMeasure['r']/1000, np.cos(np.deg2rad(markerMeasure['a'])))
            MarkerYs = np.multiply(markerMeasure['r']/1000, np.sin(np.deg2rad(markerMeasure['a'])))
            axs[idx].plot(MarkerXs, MarkerYs, 'rx', markersize=3)

            axs[idx].grid(True)
            axs[idx].axis('equal')
            axs[idx].set_xlim([-5, 5])
            axs[idx].set_ylim([-5, 5])
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
            e_a = min(np.mod(bearing_gt-bearing_z, 2*np.pi), abs(bearing_gt-bearing_z), np.mod(abs(bearing_gt-bearing_z), 2*np.pi))
        if abs(range_gt-range_z) > e_r:
            e_r = abs(range_gt-range_z)
        if idx > 0:
            x_gt_prev = dataSynchronized[idx-1][1]
            y_gt_prev = dataSynchronized[idx-1][2]
            displacement = np.sqrt((x_gt_prev-x_gt)**2+(y_gt_prev-y_gt)**2)
            dt = z[0] - dataSynchronized[idx-1][0]
            if dt == 0:
                speed = 0
            else:
                speed = displacement / dt
            if speed < 0:
                print("Get Negative speed with dt = {} [sec]".format(dt))
                exit()
            if speed > v_max:
                v_max = speed
    return e_a, e_r, v_max

# ------------------------------------------------------------------------------
def updateCalibratedPlot(data, lidar_i, calibrationParams, ax):
    groundTrue = data['gt']
    measurement = data['z'][lidar_i]
    dx = calibrationParams[0]
    dy = calibrationParams[1]
    theta = calibrationParams[2]
    rangeMaxErr = calibrationParams[4]
    markerMeasurements = ['-']

    ax.clear()
    ax.plot(groundTrue[0], groundTrue[1], 'rx', markersize=5)

    Xs = np.multiply(measurement['r']/1000, np.cos(np.deg2rad(measurement['a'])+theta)) + dx
    Ys = np.multiply(measurement['r']/1000, np.sin(np.deg2rad(measurement['a'])+theta)) + dy
    ax.plot(Xs, Ys, 'b.', markersize=1)

    for i in range(len(measurement['r'])):
        if np.sqrt((groundTrue[0]-Xs[i])**2+(groundTrue[1]-Ys[i])**2) < rangeMaxErr:
            ax.plot(Xs[i], Ys[i], 'go', markersize=3)
            markerMeasurements.append(np.deg2rad(measurement['a'][i]))
            markerMeasurements.append(measurement['r'][i]/1000)

    ax.grid(True)
    ax.axis('equal')
    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    return markerMeasurements
# ------------------------------------------------------------------------------
def saveDataAndCalibrationParam(dataSynchronized, calibrationParams):
    with open('calibrationData/calibratedData.txt', "w+") as f:
        write = csv.writer(f)
        write.writerows(dataSynchronized)
    calibrationParams = np.array(calibrationParams).reshape((len(LIDAR_PORTs),6))
    np.savetxt('calibrationData/calibrationParams.txt', calibrationParams, delimiter=',')
# ------------------------------------------------------------------------------
def calibration(filename, labelDistanceThreshold=True):
    recording = np.load(filename, allow_pickle=True)
    num_lidar = len(recording[0]['z'])
    dataCalibration = [[] for i in range(num_lidar)] # each row is a indivisual recording [timestamp-t0, gt_x, gt_y, x_measure, y_measure, bearing, range]
    calibrationParams = []
    distanceErrs = [[] for i in range(num_lidar)]
    # --------------------------------------------------------------------------
    for lidar_i in range(num_lidar):
        # Prepare data for calibration calculation
        for data in recording:
            groundTrue = data['gt']
            markerMeasure = filterMeasurement(data['z'][lidar_i], CarToLidarFoVs[lidar_i])
            # case the lidar or optitrack measurement is empty
            if len(groundTrue)*len(markerMeasure['r']) == 0:
                continue
            # append useful data to dataSynchronized for later calibration
            MarkerXs = np.multiply(markerMeasure['r']/1000, np.cos(np.deg2rad(markerMeasure['a'])))
            MarkerYs = np.multiply(markerMeasure['r']/1000, np.sin(np.deg2rad(markerMeasure['a'])))
            for i in range(len(MarkerXs)):
                if len(dataCalibration[lidar_i]) == 0:
                    t0 = data['t']
                else:
                    t0 = dataCalibration[lidar_i][0][0]
                dataCalibration[lidar_i].append([data['t']-t0, groundTrue[0], groundTrue[1], MarkerXs[i], MarkerYs[i], np.deg2rad(markerMeasure['a'])[i], markerMeasure['r'][i]/1000])
        # ----------------------------------------------------------------------
        # solve lidar pose use un-filtered dataSynchronized
        dx, dy, theta = solveLidarPose(dataCalibration[lidar_i])
        tmp_dataCalibration = copy.deepcopy(dataCalibration[lidar_i])
        for z in tmp_dataCalibration:
            pos_opti = np.array([z[1], z[2]]).reshape((2,1))
            pos_lidar = np.array([z[3]*np.cos(theta)-z[4]*np.sin(theta)+dx, z[3]*np.sin(theta)+z[4]*np.cos(theta)+dy]).reshape((2,1))
            l2error = np.linalg.norm(pos_opti-pos_lidar, ord=2)
            if labelDistanceThreshold:
                distanceErrs[lidar_i].append(l2error)
            else:
                if l2error > DistanceThreshold[lidar_i]:
                    dataCalibration[lidar_i].remove(z)
        # ----------------------------------------------------------------------
        # resolve lidar pose use filtered dataSynchronized
        if not labelDistanceThreshold:
            dx, dy, theta = solveLidarPose(dataCalibration[lidar_i])
            e_a, e_r, v_max = obtainMaximumBound(dataCalibration[lidar_i], dx, dy, theta)
            calibrationParams.append([dx.item(0), dy.item(0), theta.item(0), e_a.item(0), e_r.item(0), v_max])
            print('-----'*20)
            print("Calibration result: [dx, dy, theta] = ({}[m],{}[m],{}[deg])".format(dx, dy, np.rad2deg(theta)))
            print("Maximum error/speed bound: bearing {}[deg]; range {}[m]; speed {}[m/s]".format(np.rad2deg(e_a), e_r, v_max))
            print('-----'*20)
    # --------------------------------------------------------------------------
    # Loop through data for visual check
    dataSynchronized = []
    if labelDistanceThreshold:
        fig, axs = plt.subplots(num_lidar)
        for i in range(num_lidar):
            axs[i].hist(np.array(distanceErrs[i]), bins=len(distanceErrs[i]))
        plt.show()
    else:
        fig, axs = plt.subplots(ncols=num_lidar)
        for data in recording:
            singleData = [data['t'], data['gt'][0], data['gt'][1]] # append timestamp
            for i in range(num_lidar):
                markerMeasurements = updateCalibratedPlot(data, i, calibrationParams[i], axs[i])
                singleData.extend(markerMeasurements)
            dataSynchronized.append(singleData)
            plt.pause(0.01)
        plt.show()
        saveDataAndCalibrationParam(dataSynchronized, calibrationParams)

# ==============================================================================
if __name__ == '__main__':
    filename = 'calibrationData/calibrateRaw.npy'
    # recordData(filename)
    # replayData(filename)
    # calibration(filename, True)
    calibration(filename, False)
