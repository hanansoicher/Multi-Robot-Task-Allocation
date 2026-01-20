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

        if command.startswith("CMDS+"):
            command_string = command[5:]
            
            self._send_single_command("CLEAR")
            
            max_chunk_size = 10
            
            if not command_string:
                print(f"[RobotController] Empty command string for robot {self.id}")
                return {"status": "success", "message": "Empty command string"}
            
            # Send command string in chunks
            start = 0
            chunk_number = 0
            max_retries = 5
            
            while start < len(command_string):
                chunk_number += 1
                end = min(start + max_chunk_size, len(command_string))
                chunk = command_string[start:end]
                payload = "CMDS+" + chunk
                
                # print(f"[RobotController] Sending chunk {chunk_number} to robot {self.id}: {payload}")
                
                attempts = 0
                success = False
                
                while attempts < max_retries and not success:
                    attempts += 1
                    ret = self._send_single_command(payload)
                    
                    if ret is not None:
                        success = True
                        # print(f"[RobotController] Chunk {chunk_number} sent successfully to robot {self.id}")
                    else:
                        print(f"[RobotController] Failure {attempts} sending chunk {chunk_number} to robot {self.id}")
                        if attempts < max_retries:
                            import time
                            time.sleep(0.1)
                
                if not success:
                    print(f"[RobotController] Failed to send chunk {chunk_number} to robot {self.id} after {max_retries} attempts")
                    return None
                
                start = end
            
            # print(f"[RobotController] Successfully sent all {chunk_number} chunks to robot {self.id}")
            return {"status": "success", "message": f"Sent {chunk_number} chunks", "chunks": chunk_number}
        
        return self._send_single_command(command, need_response)

    def _send_single_command(self, command: str, need_response: bool = False):
        """Send a single command to the robot."""
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
            # print(f"[RobotController] Sending command to robot {self.id}: {command}")
            # print(f"[RobotController] Payload: {payload}")
            resp = requests.post(f"{self.API_URL}/send_command", json=payload, timeout=20)
            # print(f"[RobotController] Response status: {resp.status_code}")
            # print(f"[RobotController] Response content: {resp.text}")
            if resp.ok:
                result = resp.json()
                # print(f"[RobotController] Response JSON: {result}")
                return result
            else:
                print(f"[RobotController] HTTP request failed: {resp.status_code}")
                return None
        except Exception as e:
            print(f"[RobotController] send_command() error: {e}")
            # self.connected = False
            return None


