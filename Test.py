from pymavlink import mavutil


def connect_pixhawk(port="/dev/ttyACM0", baudrate=115200):
    try:
        # Pixhawk'a bağlantı oluştur
        master = mavutil.mavlink_connection(port, baud=baudrate)

        # Bağlantının kurulduğunu bekleyin (heartbeat bekle)
        master.wait_heartbeat()
        print("Bağlantı başarılı!")

        # Pixhawk'ı yeniden başlat (isteğe bağlı, sadece bağlantıyı test etmek için)
        master.reboot_autopilot()
        print("Pixhawk yeniden başlatıldı.")

        return master
    except Exception as e:
        print(f"Bağlantı başarısız: {e}")
        return None


if _name_ == "_main_":
    # Pixhawk'a bağlantıyı test et
    master = connect_pixhawk()

    if master:
        print("Pixhawk'a başarıyla bağlanıldı.")
    else:
        print("Pixhawk'a bağlanılamadı.")