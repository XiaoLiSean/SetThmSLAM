# parameters used in experiment

# ==============================================================================
# optitrack communication ip (win10 is the server, the ubuntu receiving data is client)
# ==============================================================================
ip_win10 = '192.168.1.5'
ip_ubuntu_pc = '192.168.1.3'
# ==============================================================================
# Local lidar ports
# ==============================================================================
LIDAR_PORT1 = '/dev/ttyUSB0'
LIDAR_PORT2 = '/dev/ttyUSB1'
LIDAR_PORT3 = '/dev/ttyUSB2'
LIDAR_PORT4 = '/dev/ttyUSB3'
LIDAR_PORTs = [LIDAR_PORT1, LIDAR_PORT2, LIDAR_PORT3]
# ==============================================================================
# Parameters used in calibration
# ==============================================================================
# [t1, t2], rmax: car is in the lidar field of view [t1,t2] within range of rmax
# where the lidar measurements is in [0, 360 deg]
CarToLidar1FoV = dict(FoVs=[[0,70], [290, 360]], rmax=1000)
CarToLidar2FoV = dict(FoVs=[[0,70], [290, 360]], rmax=1000)
CarToLidar3FoV = dict(FoVs=[[0,70], [290, 360]], rmax=1000)
CarToLidarFoVs = [CarToLidar1FoV, CarToLidar2FoV, CarToLidar3FoV]
DistanceThreshold = [[0.45, 0.55], [0.02,0.12], [0.45, 0.55]] #range of l2 error between lidar and optitrack estimation in [meter]
AngleThreshold = [5, 5, 5] #range of bearing error in degree between lidar and optitrack estimation in [meter]
