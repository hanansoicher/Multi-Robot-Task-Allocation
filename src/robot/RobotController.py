import time
import json
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from central.BluetoothAPI import BluetoothAPI

class RobotController(BluetoothAPI):
    def __init__(self, device_name: str, device_address: str, characteristic_uuid: str, reconnect_time=2):
        """Initialize the Robot controller with the Bluetooth device address."""
        super().__init__(device_address, device_name, characteristic_uuid, reconnect_time)
        self.speed = 1000
        self.calibration = 1.0

    def connect(self):
        """Connect to the robot"""
        result = super().connect()
        print(f"Connection result: {result}")
        return result.get('status') == 'connected'

    def disconnect(self):
        """Disconnect from the robot"""
        result = super().disconnect()
        return result.get('status') == 'disconnected'

    def send_command(self, command: str, need_response=False):
        """Send a command to the robot"""
        return super().send_command(command, need_response)

    def get_angle(self):
        """Get current angle - non-async version"""
        response = self.send_command("ANGLE+0", need_data=True)
        try:
            return float(response) if response else 0.0
        except (ValueError, TypeError):
            return 0.0

    def reset_angle(self):
        """Reset angle measurement - non-async version"""
        return self.send_command("ANGLE+1")

    def get_encoder_data(self):
        """Get encoder data - non-async version"""
        response = self.send_command("ENCODER+2", need_data=True)
        try:
            return float(response) if response else 0.0
        except (ValueError, TypeError):
            return 0.0

    def reset_encoder(self):
        """Reset encoder data - non-async version"""
        return self.send_command("ENCODER+3")

    def turn_right(self, angle):
        """Turn right by specified angle - non-async version"""
        return self.send_command(f"RIGHT+{angle}")

    def turn_left(self, angle):
        """Turn left by specified angle - non-async version"""
        return self.send_command(f"LEFT+{angle}")

    def move(self, duration):
        """Move forward for specified duration - non-async version"""
        return self.send_command(f"MOVE+{duration}")

    def wait(self, duration):
        """Wait for specified duration - non-async version"""
        return self.send_command(f"WAIT+{duration}")

    def status(self):
        """Get robot status - non-async version"""
        return self.send_command("STATUS", need_response=True)