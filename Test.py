import time

from pymavlink import mavutil


def main():
    master = mavutil.mavlink_connection('udp:192.168.1.100:14550')

    # master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    print("Connected")


    master.wait_heartbeat()
    print("Heartbeat received from system (system %u component %u)" % (master.target_system, master.target_component))

    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0, 1, 0, 0, 0, 0, 0, 0)

    # Arm durumunu kontrol et
    print("Waiting for the vehicle to arm...")
    master.motors_armed_wait()
    print("Motors are armed!")

    throttle = 1500
    for _ in range(4):
        master.mav.rc_channels_override_send(master.target_system, master.target_component,
                                             throttle, throttle, throttle, throttle,
                                             0, 0, 0, 0)
    print("Motors are running at throttle:", throttle)

    time.sleep(10)

    throttle = 1000
    for _ in range(4):
        master.mav.rc_channels_override_send(master.target_system, master.target_component,
                                             throttle, throttle, throttle, throttle,
                                             0, 0, 0, 0)

    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0, 0, 0, 0, 0, 0, 0, 0)
    print("Motors are stopped and disarmed.")


if __name__ == "__main__":
    main()
