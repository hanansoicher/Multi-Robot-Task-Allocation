import requests

class RobotController:
    API_URL = "http://127.0.0.1:8000"

    def __init__(self, id: int, device_address: str, characteristic_uuid: str):
        self.id = id
        self.device_address = device_address
        self.characteristic_uuid = characteristic_uuid
        self.connected = False

    def connect(self) -> bool:
        try:
            resp = requests.post(
                f"{self.API_URL}/connect",
                json={
                    "device_address": self.device_address,
                    "characteristic_uuid": self.characteristic_uuid,
                },
                timeout=10,
            )
            self.connected = resp.ok and resp.json().get("status") == "connected"
        except Exception as e:
            print(f"[RobotController] connect() failed for {self.id}: {e}")
            self.connected = False
        return self.connected

    def disconnect(self):
        try:
            requests.post(
                f"{self.API_URL}/disconnect",
                json={
                    "device_address": self.device_address,
                    "characteristic_uuid": self.characteristic_uuid,
                },
                timeout=3,
            )
        except Exception as e:
            print(f"[RobotController] disconnect() error: {e}")
        self.connected = False


    def send_command(self, command: str, need_response: bool = False):
        if not self.connected and not self.connect():
            print(f"[RobotController] Cannot send, robot {self.id} not connected.")
            return None

        try:
            payload = {
                "rc": {
                    "device_address":      self.device_address,
                    "characteristic_uuid": self.characteristic_uuid,
                },
                "cmd": {
                    "command":   command,
                    "need_data": need_response,
                },
            }
            resp = requests.post(f"{self.API_URL}/send_command", json=payload, timeout=20)
            return resp.json() if resp.ok else None
        except Exception as e:
            print(f"[RobotController] send_command() error: {e}")
            self.connected = False
            return None
