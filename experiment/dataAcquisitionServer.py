import matplotlib.pyplot as plt
import numpy as np
import threading, serial, os, socket, copy
from getchKeyPress import _Getch
from lidar import RPLidarA1
from params import LIDAR_PORT1

print('Main thread: Initialize RPLidar...\n')
lidar = RPLidarA1(LIDAR_PORT1)

# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# this function is used to update the rplidar scan [lidar.distances, lidar.angles]
def rplidar_update():
    lidar.update()
    # time.sleep(5) # wait for the update thread to finish and stop
    # lidar.stop()
    # lidar.stop_motor()
    # lidar.disconnect()
    print('Scan update thread: stop update lidar scan lidar...\n')
    return
# -------------------------------------------------------
# this function is used to detect if the user press "q" to shutdown the program
def rplidar_keyboard_shutdown():
    keypress = _Getch()
    if keypress() == 'q': # this is a blocking function
        print('Keyboard thread: shutdown lidar...\n')
        lidar.shutdown()
    return
# -------------------------------------------------------
# this function is used to send lidar scan to local program
def rplidar_tcp_server():
    print('Server thread: create server to send lidar data...\n')
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mySocket.bind(("127.0.0.1", 2333))
    print('Server thread: wait for connection...\n')
    mySocket.listen(10)
    server, addr = mySocket.accept()
    print('Server thread: connection from address {}\n'.format(addr))
    print("Server thread: press 'q' to quit data acquisition\n")
    while lidar.on:
        data = copy.deepcopy(lidar.distances + lidar.angles)
        listToStr = ' '.join([str(elem) for elem in data]) + '\n'
        server.sendall(listToStr.encode())
    print('Server thread: shutdown server\n')
    return
# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
if __name__ == '__main__':
    # os.system("sudo chmod a-rw /dev/ttyUSB0")
    t1 = threading.Thread(target=rplidar_update)
    t2 = threading.Thread(target=rplidar_keyboard_shutdown)
    t3 = threading.Thread(target=rplidar_tcp_server)
    t1.start()
    t2.start()
    t3.start()
    t1.join()
    t2.join()
    t3.join()
