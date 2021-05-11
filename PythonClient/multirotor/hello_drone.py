import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

imu_data = client.getImuData()
s = pprint.pformat(imu_data)
print("imu_data: %s" % s)
try:
    airsim.wait_key('Press any key to takeoff')
    print("Taking off...")
    client.armDisarm(True)
    client.takeoffAsync().join()

    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))

    airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
    client.moveToPositionAsync(5, 0, 0, 5).join()

    client.hoverAsync().join()

    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))

    airsim.wait_key('Press any key to reset to original state')
except Exception as error:
    print(error)
finally:
    client.reset()
    client.armDisarm(False)

    # that's enough fun for now. let's quit cleanly
    client.enableApiControl(False)
