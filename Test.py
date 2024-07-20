from pymavlink import mavutil


def connect_pixhawk(port="/dev/ttyACM0", baudrate=115200):
    try:

        master = mavutil.mavlink_connection(port, baud=baudrate)

        master.wait_heartbeat()
        print("Bağlantı başarılı!")

        master.reboot_autopilot()
        print("Pixhawk yeniden başlatıldı.")

        return master
    except Exception as e:
        print(f"Bağlantı başarısız: {e}")
        return None


if _name_ == "_main_":
    master = connect_pixhawk()

    if master:
        print("Pixhawk'a başarıyla bağlanıldı.")
    else:
        print("Pixhawk'a bağlanılamadı.")