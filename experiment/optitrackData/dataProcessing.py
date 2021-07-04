import numpy as np
import matplotlib.pyplot as plt
import copy

# ==============================================================================
MAX_SPEED = 0.2 # m/s
LIDAR_OPTITRACK_X = 7.478 # mm
LIDAR_OPTITRACK_Z = 1230.905 # mm
# ==============================================================================
# ==============================================================================
def get_time(obj):
    return obj['t']
# ==============================================================================
def processLidarData(filename):
    # sort the data by time index
    data = np.load(filename, allow_pickle=True)
    dataProcessed = copy.deepcopy(data)
    for i in range(len(data)):
        dataProcessed[i]['t'] = data[i]['t'][1]*60 + data[i]['t'][2] + float('0.'+str(data[i]['t'][3]))
    return dataProcessed
def processOptitrackData(filename):
    data = np.loadtxt(filename, delimiter=',')
    dataProcessed = []
    for i in range(data.shape[0]):
        timeStamp = data[i,0]*60 + data[i,1] + float('0.'+str(int(data[i,2])))
        dataProcessed.append(dict(t=timeStamp, x=data[i,3]-LIDAR_OPTITRACK_X, z=data[i,5]-LIDAR_OPTITRACK_Z))
    return dataProcessed
def synchronizeData():
    lidarData = processLidarData('data2.npy')
    optitrackData = processOptitrackData('test2.txt')
    data = []
    data.extend(lidarData)
    data.extend(optitrackData)
    synchronizedData = sorted(data, key=lambda x: x['t'], reverse=False)
    return synchronizedData
# ==============================================================================
if __name__ == '__main__':
    ax = plt.subplot(111)
    dataStream  = synchronizeData()
    for data in dataStream:
        # ======================================================================
        # Case optitrack data
        # ======================================================================
        if 'x' in data:
            plt.plot(data['x'], data['z'], 'ro', markersize=2)
        # ======================================================================
        # Case lidar data
        # ======================================================================
        if 'r' in data:
            num = data['r'].shape[0]
            if not num: # case measurement is empty
                continue
            idxs = [j for j in range(num) if data['r'][j] < 1000]
            if len(idxs) > 0:
                bearings = data['a'][idxs]
                ranges = data['r'][idxs]
                x = np.multiply(ranges, np.cos(bearings)).tolist()
                z = np.multiply(ranges, np.sin(bearings)).tolist()
                plt.plot(x, z, 'b.', markersize=2)
            else: # case no measurement is in the threshold
                continue
        plt.pause(0.1)
    plt.show()
    #     num = data['r'].shape[0]
    #     if not num:
    #         continue
    #     idxs = [j for j in range(num) if data[i]['r'][j] < 1000]
    #     if len(idxs) > 0:
    #         bearings = data[i]['a'][idxs]
    #         ranges = data[i]['r'][idxs]
    #         x = np.multiply(ranges, np.cos(bearings)).tolist()
    #         y = np.multiply(ranges, np.sin(bearings)).tolist()
    #         x_tmp = copy.deepcopy(x)
    #         y_tmp = copy.deepcopy(y)
    #         if 'x_prev' in locals():
    #             for j in range(len(idxs)):
    #                 dt = t - t_prev
    #                 if dt < 0:
    #                     print(data[i]['t'], data[i-1]['t'])
    #                 dx = abs(x_tmp[j]-x_prev)
    #                 dy = abs(y_tmp[j]-y_prev)
    #                 if dx > MAXSPEED*dt or dy > MAXSPEED*dt:
    #                     x.remove(x_tmp[j])
    #                     y.remove(y_tmp[j])
    #         # ======================================================================
    #         ax.clear()
    #         plt.plot(x, y, 'ro', markersize=2)
    #         ax.grid(True)
    #         ax.set_xlim(-1000, 1000)
    #         ax.set_ylim(-1000, 1000)
    #         plt.pause(0.1)
    #         # ======================================================================
    #         x_prev = np.mean(x)
    #         y_prev = np.mean(y)
    #         t_prev = t
    #
    # plt.show()
