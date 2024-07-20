from pymavlink import mavutil
import time

def main():
    # MAVLink bağlantısını başlat
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

    # Heartbeat mesajı bekle
    master.wait_heartbeat()
    print("Heartbeat received from system (system %u component %u)" % (master.target_system, master.target_component))

    # Arm komutunu gönder
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0, 1, 0, 0, 0, 0, 0, 0)

    # Arm durumunu kontrol et
    print("Waiting for the vehicle to arm...")
    master.motors_armed_wait()
    print("Motors are armed!")

    # Motorları çalıştırma (örneğin, PWM sinyali gönderme)
    # Burada örnek olarak 4 motor için hız değeri gönderiliyor
    throttle = 1500  # Örnek throttle değeri, 1000-2000 arasında olabilir
    for _ in range(4):
        master.mav.rc_channels_override_send(master.target_system, master.target_component,
                                             throttle, throttle, throttle, throttle,
                                             0, 0, 0, 0)
    print("Motors are running at throttle:", throttle)

    # Motorları bir süre çalıştır (örneğin 10 saniye)
    time.sleep(10)

    # Motorları durdur
    throttle = 1000  # Örnek minimum throttle değeri
    for _ in range(4):
        master.mav.rc_channels_override_send(master.target_system, master.target_component,
                                             throttle, throttle, throttle, throttle,
                                             0, 0, 0, 0)

    # Disarm komutunu gönder
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                 0, 0, 0, 0, 0, 0, 0, 0)
    print("Motors are stopped and disarmed.")

if __name__ == "__main__":
    main()
