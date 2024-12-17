from fastapi import FastAPI, HTTPException, Body, Request
from pydantic import BaseModel
from bleak import BleakClient
import asyncio
from typing import Dict
import traceback

app = FastAPI(debug=True)



async def catch_exceptions_middleware(request: Request, call_next):
    try:
        return await call_next(request)
    except Exception as e:
        print(traceback.format_exc())
        raise e

# Dictionary to map device address to Robot instances
robots: Dict[str, "Robot"] = {}

# Data models for receiving input from clients
class RobotCommand(BaseModel):
    command: str
    need_data: bool = False

class RobotConnection(BaseModel):
    device_address: str
    characteristic_uuid: str


class Robot:
    def __init__(self, device_address: str, characteristic_uuid: str):
        self.device_address = device_address
        self.characteristic_uuid = characteristic_uuid
        self.client = None
        self.connected = False

    async def connect(self):
        """Establish a connection to the device."""
        try:
            print("Sending connection to ", self.device_address)
            self.client = BleakClient(self.device_address)
            print("Trying to connect")
            await self.client.connect()
            print("Succesfully connected")

            self.connected = True
            return {"status": "connected"}
        except Exception as e:
            import traceback
            print(traceback.format_exc())
            raise HTTPException(status_code=500, detail=f"Failed to connect: {e}")

    async def disconnect(self):
        """Disconnect from the Bluetooth device."""
        try:
            await self.client.disconnect()
            self.connected = False
            return {"status": "disconnected"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to disconnect: {e}")

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



    async def send_command(self, command: str, need_data: bool):
        """Send a command to the connected device."""
        return await self._send_command_async(command, need_data)

# API to add a robot (connect to a device and store it in the robots dict)
@app.post("/add_robot")
async def add_robot(robot_connection: RobotConnection):
    print(robot_connection)
    if robot_connection.device_address in robots:
        return {"message": "Robot already exists in the system."}

    robot = Robot(robot_connection.device_address, robot_connection.characteristic_uuid)
    robots[robot_connection.device_address] = robot

    await robot.connect()  # Connect the robot when added
    print("[add_robot]", robots)
    return {"message": f"Robot with address {robot_connection.device_address} added and connected."}


# API to send a command to a specific robot by address
@app.post("/send_command")
async def send_command(robot_connection: RobotConnection, command: RobotCommand):
    """Endpoint to send a command to the robot."""
    if robot_connection.device_address not in robots:
        raise HTTPException(status_code=404, detail="Robot not found")

    robot = robots[robot_connection.device_address]
    return await robot.send_command(command.command, command.need_data)


# API to connect to a specific robot by address
@app.post("/connect")
async def connect(robot_connection: RobotConnection):
    """Connect to a robot by address."""

    if robot_connection.device_address not in robots or not robots[robot_connection.device_address].client.is_connected:
        robot = Robot(robot_connection.device_address, robot_connection.characteristic_uuid)
        robots[robot_connection.device_address] = robot
        return await robots[robot_connection.device_address].connect()
    
    else:
        return {"status" : "connected"}


# API to disconnect a specific robot by address
@app.post("/disconnect")
async def disconnect(robot_connection: RobotConnection):
    """Disconnect from a robot by address."""
    print("Disconnecting", robot_connection.device_address, robots)
    if robot_connection.device_address in robots:
        if robots[robot_connection.device_address].client.is_connected:
            return await robots[robot_connection.device_address].disconnect()
    else:
        return {"status" : "disconnected"}


# Clean shutdown to disconnect all robots
@app.on_event("shutdown")
async def shutdown():
    for robot in robots.values():
        if robot.client.is_connected:
            await robot.disconnect()
