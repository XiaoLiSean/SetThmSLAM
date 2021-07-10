import time, serial, threading

# ------------------------------------------------------------------------------
class RPLidarA1(object):
    '''
    https://github.com/SkoltechRobotics/rplidar
    '''
    def __init__(self, port):
        from sensors.RPLidarA1M8 import RPLidarA1M8
        self.port = port
        self.distances = [] #a list of distance measurements
        self.angles = [] # a list of angles corresponding to dist meas above
        self.lidar = RPLidarA1M8(self.port)
        self.lidar.clear_input()
        time.sleep(2)
        self.on = True

    def update(self):
        scans = self.lidar.iter_scans()
        while self.on:
            try:
                time.sleep(2) # solve error of "rplidar.RPLidarException: Wrong body size"
                for scan in scans:
                    self.distances = [item[2] for item in scan]
                    self.angles = [360-item[1] for item in scan] # make the measurement CCW while original one is clockwise
            except serial.serialutil.SerialException:
                print('RPLidarA1 class: serial.serialutil.SerialException, common when shutting down.')

    def startLidarThreaded(self):
        t = threading.Thread(target=self.update)
        t.start()

    def shutdown(self):
        self.on = False
        time.sleep(2) # wait for the update thread to finish and stop
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# In ubuntu, please sudo the access to lidar port "sudo chmod a+rw /dev/ttyUSB0"
if __name__ == '__main__':
    lidar = RPLidarA1()
    lidar.startLidarThreaded()
    while True:
        print(lidar.angles)
