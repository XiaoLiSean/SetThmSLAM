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
# ==============================================================================
# Parameters used in calibration
# ==============================================================================
# [t1, t2], rmax: car is in the lidar field of view [t1,t2] within range of rmax
# where the lidar measurements is in [0, 360 deg]
CarToLidar1FoV = dict(FoVs=[[0,90], [270, 360]], rmax=1000)
DisplacementErrorPercentage = 0.7 # used to filter the lidar measurement
