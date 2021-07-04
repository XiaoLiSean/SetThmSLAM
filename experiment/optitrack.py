from PythonClient.NatNetClient import NatNetClient
import time, copy
import numpy as np

class OptiTrack(object):
    def __init__(self, serverIPAddress=ip_win10, localIPAddress=ip_ubuntu_pc):
        # --------------------------------------------------------------------------
        # Create OptiTrack Pose Receiving CLient
        # --------------------------------------------------------------------------
        # This will create a new NatNet client
        self.streamingClient = NatNetClient(serverIPAddress=serverIPAddress, localIPAddress=localIPAddress)

        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        self.streamingClient.newFrameListener = self.receiveNewFrame
        self.streamingClient.rigidBodyListener = self.receiveRigidBodyFrame

        self.postion = []
        self.rotation = []

    def startOptiTrack(self):
        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        self.streamingClient.run()

    # ------------------------------------------------------------------------------
    # Function to Extract Pose and Velocity of the RC Car from the information
    # received from OptiTrack Server and stored at global variable "rccar_state"
    # ------------------------------------------------------------------------------
    # This is a callback function that gets connected to the NatNet client and called once per mocap frame.
    def receiveNewFrame(self, frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                        labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
        pass

    # This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
    def receiveRigidBodyFrame(self, id, position, rotation):
        self.postion = list(position)
        self.rotation = list(rotation)

# ------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
if __name__ == '__main__':
