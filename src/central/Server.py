from fastapi import FastAPI, HTTPException, Body, Request
from fastapi.concurrency import iterate_in_threadpool
from pydantic import BaseModel
from bleak import BleakClient
import asyncio
from typing import Dict
import traceback
import uvicorn

app = FastAPI(debug=True)

async def catch_exceptions_middleware(request: Request, call_next):
    try:
        return await call_next(request)
    except Exception as e:
        print(traceback.format_exc())
        raise e

app.middleware("http")(catch_exceptions_middleware)

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
        self.waiting = True

    async def connect(self):
        """Establish a connection to the device."""
        try:
            print(f"Connecting to {self.device_address}")
            self.client = BleakClient(self.device_address)
            await self.client.connect()
            print(f"Successfully connected to {self.device_address}")

            self.connected = True
            return {"status": "connected"}
        except Exception as e:
            print(traceback.format_exc())
            raise HTTPException(status_code=500, detail=f"Failed to connect: {e}")

    async def disconnect(self):
        """Disconnect from the Bluetooth device."""
        try:
            if self.client and self.client.is_connected:
                await self.client.disconnect()
                self.connected = False
                return {"status": "disconnected"}
            return {"status": "already_disconnected"}
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to disconnect: {e}")

    async def _send_command_async(self, command: str, need_data_response=False, retry_times=3):
        """Send a command to the connected Bluetooth device (internal method)."""
        if not self.client or not self.client.is_connected:
            print(f"Error: Client not connected before sending command")
            try:
                print("Attempting to reconnect...")
                await self.connect()
            except Exception as e:
                print(f"Reconnection failed: {e}")
                raise HTTPException(status_code=400, detail=f"Not connected to device and reconnection failed: {e}")

        print(f"Starting command execution: {command}")
        return_data = []
        
        for i in range(retry_times):
            try:
                # Reset waiting state
                self.waiting = True
                
                # Define notification handler before starting notifications
                async def notification_handler(sender: int, data: bytearray):
                    try:
                        message = data.decode("utf-8").strip()
                        print(f"Received notification: {message}")
                        
                        messages = message.split("\n")
                        if "COMPLETED" in messages:
                            messages.remove("COMPLETED")
                            return_data.extend(messages)
                            print("The Bluetooth device has completed the task!")
                            self.waiting = False
                    except Exception as e:
                        print(f"Error in notification handler: {e}")

                # Start notification first
                print("Starting notifications...")
                await self.client.start_notify(self.characteristic_uuid, notification_handler)
                
                # Then send command
                print(f"Writing command to characteristic: {command}")
                await self.client.write_gatt_char(self.characteristic_uuid, f"{command}\r\n".encode())

                # Wait for response with timeout
                print("Waiting for response...")
                start_time = asyncio.get_event_loop().time()
                while self.waiting:
                    await asyncio.sleep(0.1)
                    if asyncio.get_event_loop().time() - start_time > 15:
                        print(f"Command timed out: {command}")
                        break

                # Stop notification when done
                print("Stopping notifications...")
                await self.client.stop_notify(self.characteristic_uuid)
                
                if not self.waiting:
                    print(f"Command '{command}' completed successfully")
                    return "COMPLETED"
                    # if need_data_response:
                    #     return return_data[0] if return_data and len(return_data) == 1 else return_data
                    # else:
                    #     return "COMPLETED"
                else:
                    print(f"Command '{command}' timed out, retrying ({i+1}/{retry_times})")
                    self.waiting = False
                    # Try a small delay before retrying
                    await asyncio.sleep(0.5)
            except Exception as e:
                print(f"Failed to send {command} command: {e}")
                print(traceback.format_exc())
                if i == retry_times - 1:
                    raise HTTPException(status_code=500, detail=f"Failed to send command after {retry_times} attempts: {e}")
        
        return None

    async def send_command(self, command: str, need_data: bool):
        """Send a command to the connected device."""
        return await self._send_command_async(command, need_data)

@app.middleware("http")
async def log_requests(request: Request, call_next):
    body = await request.body()
    print(f"\n--- INCOMING REQUEST ---")
    print(f"URL: {request.url}")
    print(f"Method: {request.method}")
    print(f"Headers: {request.headers}")
    print(f"Body: {body.decode() if body else 'None'}")
    
    response = await call_next(request)
    
    # Create a copy of the response
    response_body = [section async for section in response.body_iterator]
    response.body_iterator = iterate_in_threadpool(iter(response_body))
    
    print(f"--- RESPONSE ---")
    print(f"Status: {response.status_code}")
    try:
        body_text = response_body[0].decode() if response_body else ""
        print(f"Body: {body_text}")
    except:
        print("Body: [Could not decode]")
    
    return response

# API to add a robot (connect to a device and store it in the robots dict)
@app.post("/add_robot")
async def add_robot(robot_connection: RobotConnection):
    print(f"Adding robot: {robot_connection}")
    if robot_connection.device_address in robots:
        return {"message": "Robot already exists in the system."}

    robot = Robot(robot_connection.device_address, robot_connection.characteristic_uuid)
    robots[robot_connection.device_address] = robot

    await robot.connect()  # Connect the robot when added
    print(f"[add_robot] Added {robot_connection.device_address}")
    return {"message": f"Robot with address {robot_connection.device_address} added and connected."}


# API to send a command to a specific robot by address
@app.post("/send_command")
async def send_command(robot_connection: RobotConnection, command: RobotCommand = Body(...)):
    """Endpoint to send a command to the robot."""
    try:
        print(f"=== COMMAND REQUEST ===")
        print(f"Address: {robot_connection.device_address}")
        print(f"Command: {command.command}")
        print(f"Need data: {command.need_data}")
        
        if robot_connection.device_address not in robots:
            print(f"Error: Robot not found in dictionary. Available robots: {list(robots.keys())}")
            raise HTTPException(status_code=404, detail="Robot not found")

        robot = robots[robot_connection.device_address]
        print(f"Found robot, connected: {robot.connected}")
        
        if not robot.connected:
            print("Attempting to reconnect robot...")
            await robot.connect()
        
        result = await robot.send_command(command.command, command.need_data)
        print(f"Command result: {result}")
        return result
    except Exception as e:
        error_detail = f"Error processing command: {str(e)}\n{traceback.format_exc()}"
        print(error_detail)
        raise HTTPException(status_code=500, detail=error_detail)

@app.post("/can_connect")
async def can_connect(robot_connection: RobotConnection):
    """Check if the device can be connected to."""
    try:
        # Create a temporary BleakClient instance to test connectivity
        client = BleakClient(robot_connection.device_address)
        await client.connect()
        await client.disconnect()
        return {"status": "can_connect", "message": f"Device {robot_connection.device_address} is reachable."}
    except Exception as e:
        return {"status": "cannot_connect", "message": f"Failed to connect to {robot_connection.device_address}: {e}"}

# API to connect to a specific robot by address
@app.post("/connect")
async def connect(robot_connection: RobotConnection):
    """Connect to a robot by address."""
    if robot_connection.device_address in robots and robots[robot_connection.device_address].client and robots[robot_connection.device_address].client.is_connected:
        return {"status": "connected"}
    
    robot = Robot(robot_connection.device_address, robot_connection.characteristic_uuid)
    robots[robot_connection.device_address] = robot
    return await robots[robot_connection.device_address].connect()

# API to disconnect a specific robot by address
@app.post("/disconnect")
async def disconnect(robot_connection: RobotConnection):
    """Disconnect from a robot by address."""
    print(f"Disconnecting {robot_connection.device_address}")
    if robot_connection.device_address in robots:
        return await robots[robot_connection.device_address].disconnect()
    else:
        return {"status": "disconnected"}

@app.post("/refresh")
async def refresh():
    """Disconnect all devices and clear the robots dictionary."""
    global robots

    try:
        # Disconnect all robots currently in the dictionary
        for robot in robots.values():
            if robot.client and robot.client.is_connected:
                await robot.disconnect()

        # Clear the dictionary
        robots.clear()

        return {"message": "All devices disconnected and dictionary cleared."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to refresh: {e}")

# Clean shutdown to disconnect all robots
@app.on_event("shutdown")
async def shutdown():
    for robot in robots.values():
        if robot.client and robot.client.is_connected:
            await robot.disconnect()

if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="debug")