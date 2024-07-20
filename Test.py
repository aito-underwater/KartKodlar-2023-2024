from pymavlink import mavutil

# MAVProxy'ye bağlan
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Bağlantı kurulduğunu doğrula
master.wait_heartbeat()
print("Bağlantı kuruldu")

# Aracı stabilize moduna geçir
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

    # Modu değiştir
    master.set_mode(mode_mapping[mode])
    print(f"Mod {mode} olarak değiştirildi.")

# Aracı arm (motorları çalıştır)
def arm_vehicle():
    master.arducopter_arm()
    print("Araç arm yapıldı.")

# Aracı disarm (motorları durdur)
def disarm_vehicle():
    master.arducopter_disarm()
    print("Araç disarm yapıldı.")

# Takeoff komutu gönder
def takeoff(altitude):
    # Arm motorları
    arm_vehicle()
    # Kalkış komutu
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude
    )
    print(f"{altitude} metreye kalkış yapılıyor.")

# Örnek kullanımlar
set_mode('STABILIZE')
arm_vehicle()
takeoff(10)
disarm_vehicle()
