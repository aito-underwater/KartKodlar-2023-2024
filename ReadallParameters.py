"""
Example of how to read all the parameters from the Autopilot with pymavlink
"""

# Disable "Broad exception" warning
# pylint: disable=W0703

import time
import sys

# Import mavutil
from pymavlink import mavutil


# Create the connection
master =  mavutil.mavlink_connection("/dev/ttyAMA0", baud=115200)
# Wait a heartbeat before sending commands
print("Waiting heartbeat")
master.wait_heartbeat()
print("heartbeat is found")
# Request all parameters
master.mav.param_request_list_send(
    master.target_system, master.target_component
)
print("Getting Params")
while True:
    time.sleep(0.01)
    try:
        message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('name: %s\tvalue: %d' % (message['param_id'],
                                       message['param_value']))
    except Exception as error:
        print(error)
        sys.exit(0)