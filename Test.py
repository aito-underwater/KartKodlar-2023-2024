import serial
import time

def check_pixhawk_connection(port='/dev/ttyACM0', baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate)
        time.sleep(2)  # Bağlantının stabil hale gelmesi için bekleyin
        if ser.is_open:
            ser.close()
            return True
    except serial.SerialException:
        return False

if _name_ == "_main_":
    if check_pixhawk_connection():
        print("Pixhawk is connected to /dev/ttyACM0")
    else:
        print("Pixhawk is NOT connected to /dev/ttyACM0")