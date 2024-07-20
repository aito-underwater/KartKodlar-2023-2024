import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

def set_target_depth(depth):
    """ Sets the target depth while in depth-hold mode. """
    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=(
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ),
        lat_int=0, lon_int=0, alt=depth,
        vx=0, vy=0, vz=0,
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
    )

def set_target_velocity(vx, vy, vz):
    """ Sets the target velocity while in depth-hold mode. """
    master.mav.set_position_target_local_ned_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0
    )

def set_target_attitude(roll, pitch, yaw):
    """ Sets the target attitude while in depth-hold mode. """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),
        master.target_system, master.target_component,
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0
    )

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
boot_time = time.time()
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Arm ArduSub autopilot and wait until confirmed
master.arducopter_arm()
master.motors_armed_wait()

# Set the desired operating mode
DEPTH_HOLD = 'ALT_HOLD'
DEPTH_HOLD_MODE = master.mode_mapping()[DEPTH_HOLD]
while not master.wait_heartbeat().custom_mode == DEPTH_HOLD_MODE:
    master.set_mode(DEPTH_HOLD)

# Set a depth target
set_target_depth(-0.3)

# Move forward at 0.5 m/s for 2 seconds (total 1 meter)
set_target_velocity(0.5, 0, 0)  # Move forward at 0.5 m/s
time.sleep(2)  # Move for 2 seconds

# Stop the vehicle
set_target_velocity(0, 0, 0)

# Clean up (disarm) at the end
master.arducopter_disarm()
master.motors_disarmed_wait()