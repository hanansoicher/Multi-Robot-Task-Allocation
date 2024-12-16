import asyncio
import json
from bleak import BleakClient, BleakScanner


class Robot:
    def __init__(self, device_address: str, characteristic_uuid: str, reconnect_time=2):
        """Initialize the Robot class with the Bluetooth device address."""
        self.device_address = device_address
        self.characteristic_uuid = characteristic_uuid
        self.reconnect_time = reconnect_time
        self.client = None
        self.connected = False

    def connect(self):
        """Synchronously connect to the BLE device."""
        return asyncio.run(self._connect())

    async def _connect(self):
        """Asynchronous method to connect to the BLE device."""
        try:
            self.client = BleakClient(self.device_address)
            await self.client.connect()
            self.connected = True
            print(f"Connected to {self.device_address}")
        except Exception as e:
            print(f"Failed to connect: {e}")
            self.connected = False

    def disconnect(self):
        """Synchronously disconnect from the BLE device."""
        return asyncio.run(self._disconnect())

    async def _disconnect(self):
        """Asynchronous method to disconnect from the BLE device."""
        if self.client:
            try:
                await self.client.disconnect()
                self.connected = False
                print(f"Disconnected from {self.device_address}")
            except Exception as e:
                print(f"Failed to disconnect: {e}")

    def send_command(self, command: str, need_response=False):
        """Synchronously send a command to the BLE device."""
        return asyncio.run(self._send_command(command, need_response))

    async def _send_command(self, command: str, need_response=False):
        """Asynchronous method to send a command to the BLE device."""
        try:
            await self.client.write_gatt_char(self.characteristic_uuid, command.encode())
            print(f"Sent command: {command}")
            if need_response:
                response = await self.client.read_gatt_char(self.characteristic_uuid)
                return response.decode("utf-8").strip()
        except Exception as e:
            print(f"Failed to send command '{command}': {e}")

    def reconnect(self, retries=3):
        """Synchronously reconnect to the BLE device."""
        return asyncio.run(self._reconnect(retries))

    async def _reconnect(self, retries=3):
        """Asynchronous method to reconnect to the BLE device."""
        for attempt in range(retries):
            await self._disconnect()
            await asyncio.sleep(self.reconnect_time)
            await self._connect()
            if self.connected:
                print("Reconnected successfully")
                return True
        print("Failed to reconnect")
        return False

    def reset_angle_data(self):
        self.send_command("ANGLE+1")

    def get_angle_data(self):
        response = self.send_command("ANGLE+0", need_response=True)
        return float(response) if response else None

    def reset_distance_data(self):
        self.send_command("ENCODER+3")

    def get_distance_data(self):
        response = self.send_command("ENCODER+2", need_response=True)
        return float(response) if response else None

    def move(self, distance):
        self.send_command(f"MOVE+{distance}")

    def turn(self, angle_in_degrees):
        self.send_command(f"TURN+{angle_in_degrees}")


def main():
    with open("devices.json", "r") as f:
        dct = json.load(f)

    device = dct["devices"][0]
    robot = Robot(device["address"], device["write_uuid"])

    robot.connect()
    if not robot.connected:
        print("Connection failed. Exiting.")
        return

    print("\n\nPHASE 1 COMPLETE\n\n")
    robot.reset_angle_data()
    print("\n\nPHASE 2 COMPLETE\n\n")
    robot.reset_distance_data()
    print("\n\nPHASE 3 COMPLETE\n\n")

    angle = robot.get_angle_data()
    print(f"Angle: {angle}")
    robot.turn(90)
    angle = robot.get_angle_data()
    print(f"Angle after turning: {angle}")

    distance = robot.get_distance_data()
    print(f"Distance: {distance}")
    robot.move(5)
    distance = robot.get_distance_data()
    print(f"Distance after moving: {distance}")

    robot.disconnect()


if __name__ == "__main__":
    main()
