import asyncio
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

    async def init(self):
        """Asynchronous initialization."""
        print("Init")
        self.client = await self._get_device_address_async(self.device_address, self.device_name)
        print(self.client)
        await self.reconnect_async(self.reconnect_time, retries=10)
        print(self.client)
        await self.reset_angle_data()
        await self.reset_distance_data()
        print("Init complete")

    async def _get_device_address_async(self, device_address, device_name):
        device = None
        if device_name:
            scanner = BleakScanner()
            device = await scanner.find_device_by_name(device_name, timeout=10.0)
            print("DEVICE", device)
            client = BleakClient(device)
            print("IN DEVICE NAME", client)

        if not device_name or device is None:
            client = BleakClient(device_address)
            print("IN ADDRESS", client)

        print("Got client")
        return client
    
    
    async def get_device_address(self, device_address, device_name):
        return await self._get_device_address_async(device_address, device_name)

    async def _connect_async(self):
        """Asynchronous method to connect to the Bluetooth device."""
        try:
            if self.client:
                await self.client.connect()
                self.connected = True
                print(f"Connected to {self.device_address}")
                return True
            else:
                print("Client is None. Cannot connect.")
        except Exception as e:
            print(f"Failed to connect to {self.device_address}: {e}")
            return False

    async def _disconnect_async(self):
        """Asynchronous method to disconnect from the Bluetooth device."""
        try:
            self.connected = False
            if self.client:
                await self.client.disconnect()
                print(f"Disconnected from {self.device_address}")
            return True
        except Exception as e:
            print(f"Failed to disconnect from {self.device_address}: {e}")
            return False

    async def reconnect_async(self, reconnect_time, retries=5):
        for i in range(retries):
            try:
                await self._disconnect_async()
                await asyncio.sleep(reconnect_time)
                await self._connect_async()

                if self.connected:
                    print("Successfully reconnected")
                    return True

                print("Retrying...")
                await asyncio.sleep(reconnect_time)
            except Exception as e:
                print(f"Reconnect attempt failed: {e}")

    async def _read_response_async(self):
        if self.client.is_connected:
            try:
                data = await self.client.read_gatt_char(self.characteristic_uuid)
                return data.decode("utf-8")
            except:
                return None
    async def _send_command_async(self, command: str, need_data_response=False, retry_times=3):
        """Send a command to the connected Bluetooth device (internal method)."""
        return_data = []
        for i in range(retry_times):
            try:
                # Assuming the command is sent as a byte array
                await self.client.write_gatt_char(self.characteristic_uuid, command.encode())
                self.waiting = True

                def notification_handler(sender: int, data: bytearray):
                    messages = data.decode("utf-8").strip().split("\n")
                    print(f"Received messages: {messages}")
                    if "COMPLETED" in messages:
                        messages.remove("COMPLETED")
                        return_data.extend(messages)
                        print("The Bluetooth device has completed the task!")
                        self.waiting = False

                await self.client.start_notify(self.characteristic_uuid, notification_handler)

                while self.waiting:
                    await asyncio.sleep(1)

                await self.client.stop_notify(self.characteristic_uuid)
                print(f"Command '{command}' sent to the device, and successfully completed")
                if need_data_response:
                    return return_data if len(return_data) > 1 else return_data[0]
                else:
                    return None
            except Exception as e:
                print(f"Failed to send {command} command: {e}")

    async def reset_angle_data(self):
        command = f"ANGLE+1"
        await self._send_command_async(command, need_data_response=False)

    async def _send_command(self, command: str, need_data = False): 
        print("sending command", command, need_data)
        return await self._send_command_async(command, need_data)

    async def is_bump_pressed(self, bump_sensor: int):
        command = f"BUMP+{bump_sensor}"
        return await self._send_command(command, need_data=True)
    
    async def is_bump_pressed_left(self):
        return await self.is_bump_pressed(0) == "TRUE"

    async def is_bump_pressed_right(self):
        return await self.is_bump_pressed(1) == "TRUE"
    
    async def bump_statuses(self):
        return [await self.is_bump_pressed_left(), await self.is_bump_pressed_right()]
    
    # Encoder Sensors 
    async def get_encoder_data(self, encoder_sensor: int):
        command = f"ENCODER+{encoder_sensor}"
        print("Get encoder data", encoder_sensor)
        return int(await self._send_command(command, need_data=True))

    async def encoder_left(self):
        return await self.get_encoder_data(0)

    async def encoder_right(self):
        return await self.get_encoder_data(1)

    async def get_distance_data(self):
        command = f"ENCODER+2"
        return float(await self._send_command(command, need_data=True))

    async def reset_distance_data(self):
        command = f"ENCODER+3"
        await self._send_command(command, need_data=False)

    # Gyro sensors
    async def get_gyro_data(self):
        command = f"GYRO+0"
        return float(await self._send_command(command, need_data=True))

    async def encoders(self):
        print("In encoders")
        l = await self.encoder_left()

        r = await self.encoder_right()
        return [l, r]

    async def status(self):
        command = f"STATUS+"
        s = await self._send_command(command, need_data=True)
        return s

    # Angle data
    async def reset_angle_data(self):
        command = f"ANGLE+1"
        await self._send_command(command, need_data=False)

    async def get_angle_data(self):
        command = f"ANGLE+0"
        return float(await self._send_command(command, need_data=True))


    async def move(self, distance: float):
        """Send a move command with a specified distance."""
        command = f"MOVE+{distance}"
        await self._send_command(command)

    async def turn(self, angle_in_degrees: float):
        """Send a move command with a specified distance."""
        command = f"TURN+{angle_in_degrees}"
        await self._send_command(command)

    async def set_speed(self, speed: int):
        command = f"SPEED+{speed}"
        await self._send_command(command)

    async def reset_distance_data(self):
        command = f"ENCODER+3"
        await self._send_command_async(command, need_data_response=False)



async def main():
    with open("devices.json", "r") as f:
        dct = json.load(f)

    d = dct["devices"][0]
    robot = Robot(d["address"], None, d["write_uuid"])
    await robot.init()

    print("\n\nPHASE 1 COMPLETE\n\n")
    await robot.reset_angle_data()
    print("\n\nPHASE 2 COMPLETE\n\n")
    await robot.reset_distance_data()
    print("\n\nPHASE 3 COMPLETE\n\n")

    r = await robot.get_angle_data()
    print(r)
    await robot.turn(90)
    
    r = await robot.get_angle_data()
    print(r)
    print("\n\n\n PHASE 2 - TURNING COMPLETE \n\n\n")

    r = await robot.get_distance_data()
    print(r)
    await robot.move(5)
    r = await robot.get_distance_data()
    print(r)
    print("\n\n\n PHASE 3 - DISTANCE COMPLETE \n\n\n")

    r_angle = await robot.get_angle_data()
    print(r_angle)

    r_enc = await robot.encoders()
    print(r_enc)

    r_dist = await robot.get_distance_data()
    print(r_dist)

    await robot.turn(180)

    print(await robot.get_angle_data())
    print(await robot.move(5))
    r_dist = await robot.get_distance_data()
    print(r_dist)




if __name__ == "__main__":
    asyncio.run(main())
