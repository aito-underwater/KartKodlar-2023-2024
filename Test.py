from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

master.wait_heartbeat()
print("Bağlantı kuruldu")

def set_mode(mode):
    # Mode listesi
    mode_mapping = {
        'STABILIZE': 0,
        'GUIDED': 4,
        'LOITER': 5,
        'RTL': 6,
        'AUTO': 10,
    }

    if mode not in mode_mapping:
        print(f"Mod {mode} geçersiz.")
        return

    master.set_mode(mode_mapping[mode])
    print(f"Mod {mode} olarak değiştirildi.")

def arm_vehicle():
    master.arducopter_arm()
    print("Araç arm yapıldı.")


def disarm_vehicle():
    master.arducopter_disarm()
    print("Araç disarm yapıldı.")

def takeoff(altitude):
    arm_vehicle()

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude
    )
    print(f"{altitude} metreye kalkış yapılıyor.")

set_mode('STABILIZE')
arm_vehicle()
takeoff(10)
disarm_vehicle()
