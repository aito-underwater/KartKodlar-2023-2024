import time

from pymavlink import mavutil


class Vehicle:
    def _init_(self, link, vehicle_id, component_id, vehicle_firmware_type, vehicle_type):
        self.link = link
        self.vehicle_id = vehicle_id
        self.component_id = component_id
        self.vehicle_firmware_type = vehicle_firmware_type
        self.vehicle_type = vehicle_type


class MultiVehicleManager:
    def _init_(self):
        self.vehicles = []
        self.ignore_vehicle_ids = set()
        self.active_vehicle = None
        self.gcs_heartbeat_enabled = True
        self.gcs_heartbeat_rate_msecs = 1000  # GCS heartbeat rate in milliseconds

    def get_vehicle_by_id(self, vehicle_id):
        for vehicle in self.vehicles:
            if vehicle.vehicle_id == vehicle_id:
                return vehicle
        return None

    def _vehicle_heartbeat_info(self, link, vehicle_id, component_id, vehicle_firmware_type, vehicle_type):

        if vehicle_type == 0 and vehicle_firmware_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            vehicle_type = mavutil.mavlink.MAV_TYPE_QUADROTOR

        if len(self.vehicles) > 0 and not self.multi_vehicle_enabled():
            return
        vehicle = Vehicle(link, vehicle_id, component_id, vehicle_firmware_type, vehicle_type)
        self.vehicles.append(vehicle)
        self._send_gcs_heartbeat()
        self.set_active_vehicle(vehicle)

    def _send_gcs_heartbeat(self):
        if not self.gcs_heartbeat_enabled:
            return
        while True:
            for vehicle in self.vehicles:
                mavlink_connection = vehicle.link
                if mavlink_connection:
                    mavlink_connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,  # MAV_TYPE
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,  # MAV_AUTOPILOT
                        mavutil.mavlink.MAV_MODE_MANUAL_ARMED,  # MAV_MODE
                        0,  # custom mode
                        mavutil.mavlink.MAV_STATE_ACTIVE  # MAV_STATE
                    )
            time.sleep(self.gcs_heartbeat_rate_msecs / 1000.0)

    def set_active_vehicle(self, vehicle):
        self.active_vehicle = vehicle


link = mavutil.mavlink_connection('udp:127.0.0.1:14550')
manager = MultiVehicleManager()
manager._vehicle_heartbeat_info(link, 1, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                                mavutil.mavlink.MAV_TYPE_QUADROTOR)
import threading

heartbeat_thread = threading.Thread(target=manager._send_gcs_heartbeat)
heartbeat_thread.daemon = True
heartbeat_thread.start()
