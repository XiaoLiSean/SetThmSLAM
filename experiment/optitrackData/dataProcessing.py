import numpy as np
import matplotlib.pyplot as plt
import copy

MAXSPEED = 0.2 # m/s
def get_time(obj):
    return obj['t']
# ==============================================================================
# sort the data by time index
data = np.load('data1.npy', allow_pickle=True)
for i in range(len(data)):
    t = data[i]['t'][0]*60*60 + data[i]['t'][1]*60 + data[i]['t'][2] + float('0.'+str(data[i]['t'][3]))
    data[i]['t'] = t
data = sorted(data, key=lambda x: x['t'], reverse=False)
# ==============================================================================
ax = plt.subplot(111)
measurement = []
for i in range(len(data)):
    num = data[i]['r'].shape[0]
    if not num:
        continue
    idxs = [j for j in range(num) if data[i]['r'][j] < 1000]
    if len(idxs) > 0:
        bearings = data[i]['a'][idxs]
        ranges = data[i]['r'][idxs]
        x = np.multiply(ranges, np.cos(bearings)).tolist()
        y = np.multiply(ranges, np.sin(bearings)).tolist()
        x_tmp = copy.deepcopy(x)
        y_tmp = copy.deepcopy(y)
        if 'x_prev' in locals():
            for j in range(len(idxs)):
                dt = t - t_prev
                if dt < 0:
                    print(data[i]['t'], data[i-1]['t'])
                dx = abs(x_tmp[j]-x_prev)
                dy = abs(y_tmp[j]-y_prev)
                if dx > MAXSPEED*dt or dy > MAXSPEED*dt:
                    x.remove(x_tmp[j])
                    y.remove(y_tmp[j])
        # ======================================================================
        ax.clear()
        plt.plot(x, y, 'ro', markersize=2)
        ax.grid(True)
        ax.set_xlim(-1000, 1000)
        ax.set_ylim(-1000, 1000)
        plt.pause(0.1)
        # ======================================================================
        x_prev = np.mean(x)
        y_prev = np.mean(y)
        t_prev = t

plt.show()
