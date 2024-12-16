import time
import json
from bleak import BleakClient, BleakScanner

class Robot:
    def __init__(self, device_address: str, device_name: str, characteristic_uuid: str, reconnect_time=2):
        """Initialize the Robot class with the Bluetooth device address."""
        self.device_address = device_address
        self.characteristic_uuid = characteristic_uuid
        self.device_name = device_name
        self.client = None
        self.speed = 1000
        self.waiting = False
        self.connected = False
        self.reconnect_time = reconnect_time

        print("address", device_address, "name", device_name)

    def init(self):
        """Synchronous initialization."""
        print("Init")
        self.client = self._get_device_address(self.device_address, self.device_name)
        self.reconnect(self.reconnect_time, retries=10)
        self.reset_angle_data()
        self.reset_distance_data()
        print("Init complete")

    def _get_device_address(self, device_address, device_name):
        device = None
        if device_name:
            scanner = BleakScanner()
            device = scanner.find_device_by_name(device_name, timeout=10.0)
            print("DEVICE", device)
            client = BleakClient(device)
            print("IN DEVICE NAME", client)

        if not device_name or device is None:
            client = BleakClient(device_address)
            print("IN ADDRESS", client)

        print("Got client")
        return client

    def connect(self):
        """Synchronous method to connect to the Bluetooth device."""
        try:
            if self.client:
                self.client.connect()
                self.connected = True
                print(f"Connected to {self.device_address}")
                return True
            else:
                print("Client is None. Cannot connect.")
        except Exception as e:
            print(f"Failed to connect to {self.device_address}: {e}")
            return False

    def disconnect(self):
        """Synchronous method to disconnect from the Bluetooth device."""
        try:
            self.connected = False
            if self.client:
                self.client.disconnect()
                print(f"Disconnected from {self.device_address}")
            return True
        except Exception as e:
            print(f"Failed to disconnect from {self.device_address}: {e}")
            return False

    def reconnect(self, reconnect_time, retries=5):
        for i in range(retries):
            try:
                self.disconnect()
                time.sleep(reconnect_time)
                self.connect()

                if self.connected:
                    print("Successfully reconnected")
                    return True

                print("Retrying...")
                time.sleep(reconnect_time)
            except Exception as e:
                print(f"Reconnect attempt failed: {e}")

    def _read_response(self):
        if self.client.is_connected:
            try:
                data = self.client.read_gatt_char(self.characteristic_uuid)
                return data.decode("utf-8")
            except:
                return None

    def _send_command(self, command: str, need_data=False, retry_times=3):
        """Send a command to the connected Bluetooth device (internal method)."""
        return_data = []
        for i in range(retry_times):
            try:
                # Assuming the command is sent as a byte array
                self.client.write_gatt_char(self.characteristic_uuid, command.encode())
                self.waiting = True

                def notification_handler(sender: int, data: bytearray):
                    messages = data.decode("utf-8").strip().split("\n")
                    print(f"Received messages: {messages}")
                    if "COMPLETED" in messages:
                        messages.remove("COMPLETED")
                        return_data.extend(messages)
                        print("The Bluetooth device has completed the task!")
                        self.waiting = False

                self.client.start_notify(self.characteristic_uuid, notification_handler)

                while self.waiting:
                    time.sleep(1)

                self.client.stop_notify(self.characteristic_uuid)
                print(f"Command '{command}' sent to the device, and successfully completed")
                if need_data:
                    return return_data if len(return_data) > 1 else return_data[0]
                else:
                    return None
            except Exception as e:
                print(f"Failed to send {command} command: {e}")

    def reset_angle_data(self):
        command = f"ANGLE+1"
        self._send_command(command, need_data=False)

    def _send_command_sync(self, command: str, need_data=False):
        print("sending command", command, need_data)
        return self._send_command(command, need_data)

    def is_bump_pressed(self, bump_sensor: int):
        command = f"BUMP+{bump_sensor}"
        return self._send_command_sync(command, need_data=True)

    def is_bump_pressed_left(self):
        return self.is_bump_pressed(0) == "TRUE"

    def is_bump_pressed_right(self):
        return self.is_bump_pressed(1) == "TRUE"

    def bump_statuses(self):
        return [self.is_bump_pressed_left(), self.is_bump_pressed_right()]

    # Encoder Sensors 
    def get_encoder_data(self, encoder_sensor: int):
        command = f"ENCODER+{encoder_sensor}"
        print("Get encoder data", encoder_sensor)
        return int(self._send_command_sync(command, need_data=True))

    def encoder_left(self):
        return self.get_encoder_data(0)

    def encoder_right(self):
        return self.get_encoder_data(1)

    def get_distance_data(self):
        command = f"ENCODER+2"
        return float(self._send_command_sync(command, need_data=True))

    def reset_distance_data(self):
        command = f"ENCODER+3"
        self._send_command(command, need_data=False)

    # Gyro sensors
    def get_gyro_data(self):
        command = f"GYRO+0"
        return float(self._send_command_sync(command, need_data=True))

    def encoders(self):
        print("In encoders")
        l = self.encoder_left()

        r = self.encoder_right()
        return [l, r]

    def status(self):
        command = f"STATUS+"
        s = self._send_command_sync(command, need_data=True)
        return s

    # Angle data
    def reset_angle_data(self):
        command = f"ANGLE+1"
        self._send_command(command, need_data=False)

    def get_angle_data(self):
        command = f"ANGLE+0"
        return float(self._send_command_sync(command, need_data=True))

    def move(self, distance: float):
        """Send a move command with a specified distance."""
        command = f"MOVE+{distance}"
        self._send_command(command)

    def turn(self, angle_in_degrees: float):
        """Send a move command with a specified distance."""
        command = f"TURN+{angle_in_degrees}"
        self._send_command(command)

    def set_speed(self, speed: int):
        command = f"SPEED+{speed}"
        self._send_command(command)


def main():
    with open("devices.json", "r") as f:
        dct = json.load(f)

    d = dct["devices"][0]
    robot = Robot(d["address"], None, d["write_uuid"])
    robot.init()

    print("\n\nPHASE 1 COMPLETE\n\n")
    robot.reset_angle_data()
    print("\n\nPHASE 2 COMPLETE\n\n")
    robot.reset_distance_data()
    print("\n\nPHASE 3 COMPLETE\n\n")

    r = robot.get_angle_data()
    print(r)
    robot.turn(90)

    r = robot.get_angle_data()
    print(r)
    print("\n\n\n PHASE 2 - TURNING COMPLETE \n\n\n")

    r = robot.get_distance_data()
    print(r)
    robot.move(5)
    r = robot.get_distance_data()
    print(r)
    print("\n\n\n PHASE 3 - DISTANCE COMPLETE \n\n\n")

    r_angle = robot.get_angle_data()
    print(r_angle)

    r_enc = robot.encoders()
    print(r_enc)

    r_dist = robot.get_distance_data()
    print(r_dist)

    robot.turn(180)

    print(robot.get_angle_data())
    print(robot.move(5))
    r_dist = robot.get_distance_data()
    print(r_dist)


if __name__ == "__main__":
    main()
