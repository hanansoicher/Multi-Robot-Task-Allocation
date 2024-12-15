import asyncio
import time
import json 
from bleak import BleakClient, BleakScanner


# Device address
device_address = "B0:D2:78:32:EA:6C"
#device_address = "8386512F-23A5-9409-4353-C3E8EB1C6C4B" # Henrik
# UUID for the writable characteristic
write_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"


class Robot:
    def __init__(self, device_address: str, device_name: str, characteristic_uuid: str, reconnect_time = 2):
        """Initialize the Robot class with the Bluetooth device address."""
        self.device_address = device_address
        self.characteristic_uuid = characteristic_uuid
        self.client = self.get_device_address(device_address, device_name)
        self.speed = 1000 
        self.waiting = False
        self.connected = False
        self.reconnect_time = reconnect_time

        print("address", device_address, "name", device_name)
        self.init()

    def init(self):
        print("Init")

        self.reconnect(self.reconnect_time, retries=10)
        # self.reconnect(self.reconnect_time)
        self.reset_angle_data()
        self.reset_distance_data()
        print("Init complete")

    async def _get_device_address_async(self, device_address, device_name):
        device = None
        if device_name:
            scanner = BleakScanner()
            device = await scanner.find_device_by_name(device_name, timeout=10.0)
            print("DEVICE", device)
            client = BleakClient(device)
            print("IN DEVICE NAME", client)

        if not device_name or device == None:
            client = BleakClient(device_address)
            print("IN ADDRESS", client)
        
        print("Got client")
        return client
        

    def get_device_address(self, device_address, device_name):
        return asyncio.run(self._get_device_address_async(device_address, device_name))



    async def _connect_async(self):
        """Asynchronous method to connect to the Bluetooth device."""
        try:
            await self.client.connect()
            self.connected = True
            print(f"Connected to {self.device_address}")
            return True
        except Exception as e:
            print(f"Failed to connect to {self.device_address}: {e}")
            return False

    async def _disconnect_async(self):
        """Asynchronous method to connect to the Bluetooth device."""
        try:
            self.connected = False
            await self.client.disconnect()
            print(f"Disconnected from {self.device_address}")
            return True
        except Exception as e:
            print(f"Failed to connect to {self.device_address}: {e}")
            return False 


    def connect(self):
        """Connect to the Bluetooth device."""
        asyncio.run(self._connect_async())
    
    def disconnect(self):
        """Disconnect from the Bluetooth device."""
        asyncio.run(self._disconnect_async())
    
    def reconnect(self, reconnect_time, retries = 5):
        for i in range(retries):
            try:
                self.disconnect()
                time.sleep(reconnect_time)
                self.connect()

                print("Done")
                if self.connected:
                    print("Successfully reconnected")
                    return True

                print("Sleeping")
                time.sleep(reconnect_time)
            except Exception as e:
                print("Exception ", e)

    async def reconnect_async(self, reconnect_time):
        await self._disconnect_async()
        await asyncio.sleep(reconnect_time)
        await self._connect_async()

    async def _read_response_async(self):
        if self.client.is_connected:
            try:
                data = await self.client.read_gatt_char(self.characteristic_uuid)
                return data.decode("utf-8")
            except:
                return None

    
    async def _send_command_async(self, command: str, need_data_response = False, retry_times = 3):
        """Send a command to the connected Bluetooth device (internal method)."""
        
        return_data = []
        # while not self.connected:
        #     await self.reconnect_async(self.reconnect_time)

        for i in range(retry_times):
            try:
                # Assuming the command is sent as a byte array
                await self.client.write_gatt_char(self.characteristic_uuid, command.encode())
                # ack = None

                self.waiting = True
                def notification_handler(sender: int, data: bytearray):

                    messages = data.decode("utf-8").strip().split('\n')
                    print(f"Received messages: {messages}")
                    if "COMPLETED" in messages:
                        messages.remove("COMPLETED")

                        print([type(m) for m in messages], return_data, messages)
                        return_data.extend(messages)
                        print("The Bluetooth device has completed the task!")
                        self.waiting = False

                await self.client.start_notify(self.characteristic_uuid, notification_handler)

                # Continue listening until the message "COMPLETED" is received

                while self.waiting:
                    await asyncio.sleep(1)

                # Stop listening (optional, you may choose when to stop)
                await self.client.stop_notify(self.characteristic_uuid)
                
                print(f"Command '{command}' sent to the device, and successfully completed")
                if need_data_response:
                    return return_data if len(return_data) > 1 else return_data[0] 
                else:
                    return None
            except Exception as e:
                print(f"Failed to send {command} command: {e}")
    
    def _send_command(self, command: str, need_data = False): 
        print("sending command", command, need_data)
        return asyncio.run(self._send_command_async(command, need_data))

    # Bump Sensors 
    def is_bump_pressed(self, bump_sensor: int):
        command = f"BUMP+{bump_sensor}"
        return self._send_command(command, need_data=True)
    
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
        return int(self._send_command(command, need_data=True))

    def encoder_left(self):
        return self.get_encoder_data(0)

    def encoder_right(self):
        return self.get_encoder_data(1)

    def get_distance_data(self):
        command = f"ENCODER+2"
        return float(self._send_command(command, need_data=True))

    def reset_distance_data(self):
        command = f"ENCODER+3"
        self._send_command(command, need_data=False)

    # Gyro sensors
    def get_gyro_data(self):
        command = f"GYRO+0"
        return float(self._send_command(command, need_data=True))

    def encoders(self):
        print("In encoders")
        l = self.encoder_left()

        r = self.encoder_right()
        return [l, r]

    def status(self):
        command = f"STATUS+"
        s = self._send_command(command, need_data=True)
        return s

    # Angle data
    def reset_angle_data(self):
        command = f"ANGLE+1"
        self._send_command(command, need_data=False)

    def get_angle_data(self):
        command = f"ANGLE+0"
        return float(self._send_command(command, need_data=True))


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
    with open('devices.json', 'r') as f:
        dct = json.load(f)


    d = dct["devices"][0]
    robot = Robot(d["address"], "149_Proj_G8", d["write_uuid"])

    print(robot.bump_statuses())
    print("Got status", robot.status())
    print("Got encoder ticks", robot.encoders())
    print("\n\n\n PHASE 1 COMPLETE \n\n\n")

    # robot.move(5)
    # print("Got encoder ticks", robot.encoders())

    
    print(robot.get_angle_data())
    robot.turn(90)
    print(robot.get_angle_data())
    print("\n\n\n PHASE 2 - TURNING COMPLETE \n\n\n")


    print(robot.get_distance_data())
    robot.move(5)
    print(robot.get_distance_data())
    print("\n\n\n PHASE 3 - DISTANCE COMPLETE \n\n\n")

    # import random 

    # for i in range(10):
    #     e = random.randint(1, 10)        
    #     if e <= 5 :
    #         robot.move(1)  # Move by 1 units
    #     else:
    #         robot.turn(15)  # Turn 90 degrees right

    robot.disconnect()


main()