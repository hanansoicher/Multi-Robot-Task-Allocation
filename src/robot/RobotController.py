import asyncio
import logging
import time
import json
from bleak import BleakClient, BleakScanner

class RobotController:
    def __init__(self, device_name: str, device_address: str, characteristic_uuid: str, reconnect_time=2):
        """Initialize the Robot class with the Bluetooth device address."""
        self.device_name = device_name
        self.device_address = device_address
        self.characteristic_uuid = characteristic_uuid
        self.client = None
        self.command_queue = asyncio.Queue()
        self.speed = 1000
        self.waiting = False
        self.connected = False
        self.reconnect_time = reconnect_time
        self.logger = logging.getLogger(__name__)
        self.response_future = None
        self.command_processor_task = None
        print("address", device_address, "name", device_name)

    async def connect(self, retries = 3):
        """Connect to the robot with retry mechanism"""
        for i in range(retries):
            try:
                self.client = BleakClient(self.device_address)
                await self.client.connect()
                self.logger.info(f"Connected to {self.device_address}")
                self.command_processor_task = asyncio.create_task(self._process_command_queue())
                return True
            except Exception as e:
                self.logger.error(f"Connection attempt {i + 1} failed: {str(e)}")
                await asyncio.sleep(1)
        return False
    
    async def disconnect(self):
        """Disconnect from the robot and cleanup"""
        if self.command_processor_task:
            self.command_processor_task.cancel()
            try:
                await self.command_processor_task
            except asyncio.CancelledError:
                pass
        
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            self.logger.info(f"Disconnected from {self.device_address}")

    async def _process_command_queue(self):
        """Process commands from the queue"""
        while True:
            try:
                command, need_response = await self.command_queue.get()
                await self._execute_command(command, need_response)
            except asyncio.CancelledError:
                break
            except Exception as e:
                self.logger.error(f"Error processing command: {str(e)}")

    def notification_handler(self, sender: int, data: bytearray):
        """Handle incoming notifications from the robot"""
        try:
            messages = data.decode("utf-8").strip().split("\n")
            self.logger.debug(f"Received messages: {messages}")
            
            if "COMPLETED" in messages:
                messages.remove("COMPLETED")
                if self.response_future and not self.response_future.done():
                    if messages:
                        self.response_future.set_result(messages[0])
                    else:
                        self.response_future.set_result(None)
        except Exception as e:
            self.logger.error(f"Error in notification handler: {str(e)}")
            if self.response_future and not self.response_future.done():
                self.response_future.set_exception(e)

    async def _execute_command(self, command: str, need_response, retries = 3):
        """Execute a command with retries and timeout"""
        for attempt in range(retries):
            try:
                if need_response:
                    self.response_future = asyncio.Future()
                
                await self.client.start_notify(self.characteristic_uuid, self.notification_handler)
                await self.client.write_gatt_char(self.characteristic_uuid, command.encode())
                
                if need_response:
                    response = await asyncio.wait_for(self.response_future, timeout=2.0)
                    await self.client.stop_notify(self.characteristic_uuid)
                    return response
                
                await self.client.stop_notify(self.characteristic_uuid)
                return None
                
            except asyncio.TimeoutError:
                self.logger.warning(f"Command timed out, attempt {attempt + 1}/{retries}")
                continue
            except Exception as e:
                self.logger.error(f"Command execution failed: {str(e)}")

    async def send_command(self, command: str, need_response: bool = False):
            """Add a command to the queue"""
            await self.command_queue.put((command, need_response))
        
    async def get_angle(self) -> float:
        """Get current angle"""
        response = await self._execute_command("ANGLE+0", True)
        return float(response) if response else 0.0

    async def reset_angle(self):
        """Reset angle measurement"""
        await self.send_command("ANGLE+1")

    async def get_encoder_data(self) -> float:
        """Get encoder data"""
        response = await self._execute_command("ENCODER+2", True)
        return float(response) if response else 0.0

    async def reset_encoder(self):
        """Reset encoder data"""
        await self.send_command("ENCODER+3")




