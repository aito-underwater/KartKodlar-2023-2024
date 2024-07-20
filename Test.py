import time
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for the first heartbeat
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        break

# Request parameter value
master.mav.param_request_read_send(
    master.target_system, master.target_component,
    b'SURFACE_DEPTH',
    -1
)

# Print old parameter value
while True:
    message = master.recv_match(type='PARAM_VALUE', blocking=True)
    if message and message.param_id == b'SURFACE_DEPTH':
        print('name: %s\tvalue: %f' % (message.param_id.decode("utf-8"), message.param_value))
        break

time.sleep(1)

# Set parameter value
master.mav.param_set_send(
    master.target_system, master.target_component,
    b'SURFACE_DEPTH',
    -12.0,  # New parameter value (float) to be set
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)

# Wait for ACK (PARAM_VALUE message confirming the change)
while True:
    message = master.recv_match(type='PARAM_VALUE', blocking=True)
    if message and message.param_id == b'SURFACE_DEPTH':
        print('name: %s\tvalue: %f' % (message.param_id.decode("utf-8"), message.param_value))
        break

time.sleep(1)

# Request parameter value again to confirm the change
master.mav.param_request_read_send(
    master.target_system, master.target_component,
    b'SURFACE_DEPTH',
    -1
)

# Print new value in RAM
while True:
    message = master.recv_match(type='PARAM_VALUE', blocking=True)
    if message and message.param_id == b'SURFACE_DEPTH':
        print('name: %s\tvalue: %f' % (message.param_id.decode("utf-8"), message.param_value))
        break

# Close the connection
master.close()