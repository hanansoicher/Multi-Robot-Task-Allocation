from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from bleak import BleakClient, BleakError, BleakScanner
import uvicorn
from typing import Dict, Optional
import asyncio


app = FastAPI(title="BLE–Robot Bridge", version="1.0")


class RobotConnection(BaseModel):
    device_address: str
    characteristic_uuid: str


class Command(BaseModel):
    command: str
    need_data: bool = False


class Robot:
    """Wraps a BleakClient instance and hides reconnect logic."""

    def __init__(self, address: str, char_uuid: str):
        self.address = address
        self.uuid = char_uuid
        self.client: Optional[BleakClient] = None
        self.response_buffer = ""

    async def ensure_connected(self):
        if self.client and self.client.is_connected:
            return
        # scanner = BleakScanner()
        # devices = await scanner.discover(15)
        # for device in devices:
        #     if device.address == self.address:
        #         print(device.name)
        #         self.client = BleakClient(device)
        #         break
        
        self.client = BleakClient(self.address)
        try:
            await self.client.connect()
            await self.client.start_notify(self.uuid, self._notification_handler)
        except BleakError as err:
            raise HTTPException(status_code=500, detail=f"BLE connect failed: {err}")

    def _notification_handler(self, sender, data):
        try:
            response = data.decode('utf-8')
            self.response_buffer += response
            print(f"[Robot {self.address}] Received: {response}")
        except Exception as e:
            print(f"[Robot {self.address}] Error decoding response: {e}")

    async def write(self, payload: str):
        await self.ensure_connected()
        
        self.response_buffer = ""
        
        await self.client.write_gatt_char(self.uuid, f"{payload}\r\n".encode())
        
        max_wait_time = 15
        start_time = asyncio.get_event_loop().time()
        
        while asyncio.get_event_loop().time() - start_time < max_wait_time:
            if "COMPLETED" in self.response_buffer or "FAILED" in self.response_buffer:
                break
            await asyncio.sleep(0.1)
        
        response_data = {
            "status": "sent",
            "response": self.response_buffer.strip(),
        }
        
        return response_data

    async def close(self):
        if self.client and self.client.is_connected:
            await self.client.disconnect()
        self.client = None


robots: Dict[str, Robot] = {}


@app.post("/connect")
async def connect(rc: RobotConnection):
    robot = robots.get(rc.device_address) or Robot(rc.device_address, rc.characteristic_uuid)
    await robot.ensure_connected()
    robots[rc.device_address] = robot
    return {"status": "connected"}


@app.post("/send_command")
async def send_command(rc: RobotConnection, cmd: Command):
    robot = robots.get(rc.device_address)
    if not robot:
        raise HTTPException(status_code=404, detail="Robot not connected")
    return await robot.write(cmd.command)


@app.post("/disconnect")
async def disconnect(rc: RobotConnection):
    robot = robots.pop(rc.device_address, None)
    if robot:
        await robot.close()
    return {"status": "disconnected"}


if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="info")
