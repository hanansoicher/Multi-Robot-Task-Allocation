import asyncio
import time
from bleak import BleakClient

# Device address
device_address = "B0:D2:78:32:EA:6C"
# UUID for the writable characteristic
write_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"

class Robot:
    def __init__(self, device_address: str, characteristic_uuid: str):
        """Initialize the Robot class with the Bluetooth device address."""
        self.device_address = device_address
        self.characteristic_uuid = characteristic_uuid
        self.client = BleakClient(self.device_address)
        self.speed = 1000 
        self.waiting = False

    async def _connect_async(self):
        """Asynchronous method to connect to the Bluetooth device."""
        try:
            await self.client.connect()
            print(f"Connected to {self.device_address}")
        except Exception as e:
            print(f"Failed to connect to {self.device_address}: {e}")

    def connect(self):
        """Connect to the Bluetooth device."""
        asyncio.run(self._connect_async())
    
    async def _read_response_async(self):
        if self.client.is_connected:
            try:
                data = await self.client.read_gatt_char(self.characteristic_uuid)
                return data
            except:
                return None

    
    async def _send_command_async(self, command: str):
        """Send a command to the connected Bluetooth device (internal method)."""
        
        if self.client.is_connected:
            try:
                # Assuming the command is sent as a byte array
                await self.client.write_gatt_char(self.characteristic_uuid, command.encode())
                # ack = None

                self.waiting = True
                def notification_handler(sender: int, data: bytearray):

                    message = data.decode("utf-8")
                    print(f"Received message: {message}")
                    if message == "COMPLETED":
                        print("The Bluetooth device has completed the task!")
                        self.waiting = False

                await self.client.start_notify(self.characteristic_uuid, notification_handler)

                # Continue listening until the message "COMPLETED" is received

                while self.waiting:
                    await asyncio.sleep(1)

                # Stop listening (optional, you may choose when to stop)
                await self.client.stop_notify(self.characteristic_uuid)
                
                print(f"Command '{command}' sent to the device, and successfully completed")
            except Exception as e:
                print(f"Failed to send command: {e}")
        else:
            print("Device not connected.")
    
    def _send_command(self, command: str): 
        asyncio.run(self._send_command_async(command))

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

    def disconnect(self):
        """Disconnect from the Bluetooth device."""
        try:
            self.client.disconnect()
            print(f"Disconnected from {self.device_address}")
        except Exception as e:
            print(f"Failed to disconnect: {e}")

def main():
    robot = Robot(device_address, write_uuid)  # Replace with your Bluetooth device address
    robot.connect()

    import random 

    for i in range(10):
        e = random.randint(1, 10)        
        if e <= 5 :
            robot.move(1)  # Move by 1 units
        else:
            robot.turn(15)  # Turn 90 degrees right

    robot.disconnect()


main()