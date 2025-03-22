import requests
import json
import time

class BluetoothAPI:
    API_URL = "http://127.0.0.1:8000"

    def __init__(self, device_address: str, device_name: str, characteristic_uuid: str, reconnect_time=2, api_url: str = "http://127.0.0.1:8000"):
        self.device_address = device_address
        self.device_name = device_name
        self.name = device_name
        self.characteristic_uuid = characteristic_uuid
        self.api_url = api_url
        self.device_data_json = {
            "device_address": self.device_address,
            "characteristic_uuid": self.characteristic_uuid
        }
        self.connected = False

    @staticmethod
    def refresh():
        """Remove all robot connections from the server."""
        response = requests.post(f"{BluetoothAPI.API_URL}/refresh")
        return response.json()

    @staticmethod
    def get_device_data(name):
        """Get device info from the devices.json file by name."""
        data = None
        with open('devices.json', 'r') as f:
            dct = json.load(f)
            for d in dct['devices']:
                if d['name'] == name:
                    data = d
                    break
        return data

    @staticmethod
    def can_connect(device_address, characteristic_uuid):
        """Synchronous wrapper for the can connect API."""
        try:
            response = requests.post(f"{BluetoothAPI.API_URL}/can_connect", json={
                "device_address": device_address,
                "characteristic_uuid": characteristic_uuid,
            })
            r = response.json()
            print(f"Can connect response: {r}")
            return r['status'] == 'can_connect'
        except Exception as e:
            print(f"Error checking connection: {e}")
            return False

    def connect(self):
        """Synchronous wrapper for the connect API."""
        try:
            print(f"Trying to connect to {self.device_name} at {self.device_address} with UUID {self.characteristic_uuid}")
            response = requests.post(f"{self.api_url}/connect", json={
                "device_address": self.device_address,
                "characteristic_uuid": self.characteristic_uuid,
            })
            result = response.json()
            print(f"Full connection response: {result}")
            self.connected = result.get('status') == 'connected'
            return result
        except Exception as e:
            print(f"Error connecting: {e}")
            self.connected = False
            return {"status": "error", "message": str(e)}

    def disconnect(self):
        """Synchronous wrapper for the disconnect API."""
        try:
            response = requests.post(f"{self.api_url}/disconnect", json={
                "device_address": self.device_address,
                "characteristic_uuid": self.characteristic_uuid,
            })
            result = response.json()
            self.connected = False
            return result
        except Exception as e:
            print(f"Error disconnecting: {e}")
            return {"status": "error", "message": str(e)}

    def add_robot(self):
        """Register a robot with the server."""
        try:
            response = requests.post(f"{self.api_url}/add_robot", json={
                "device_address": self.device_address,
                "characteristic_uuid": self.characteristic_uuid,
            })
            result = response.json()
            self.connected = True
            return result
        except Exception as e:
            print(f"Error adding robot: {e}")
            return {"status": "error", "message": str(e)}

    def send_command(self, command: str, need_data: bool = False):
        """Synchronous wrapper for sending commands."""
        try:
            if not self.connected:
                print("Not connected, attempting to connect...")
                self.connect()
                
            data = {"command": command, "need_data": need_data}
            response = requests.post(f"{self.api_url}/send_command", json={
                "robot_connection": {
                    "device_address": self.device_address,
                    "characteristic_uuid": self.characteristic_uuid,
                },
                "command": data
            })
            
            if response.status_code != 200:
                print(f"Error sending command: {response.text}")
                return None
            
            return response.json()
        except Exception as e:
            print(f"Error sending command '{command}': {e}")
            return None