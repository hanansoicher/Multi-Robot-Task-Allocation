import requests

class Robot:
    def __init__(self, device_address: str, device_name: str, characteristic_uuid: str, reconnect_time = 2, api_url: str = "http://127.0.0.1:8000"):
        self.device_address = device_address
        self.device_name = device_name
        self.characteristic_uuid = characteristic_uuid
        self.api_url = api_url

    def init(self):
        """Asynchronous initialization."""
        print("Init")
        self.disconnect()
        self.add_robot()
        self.connect()
        self.reset_angle_data()
        self.reset_distance_data()
        print("Init complete")

    def add_robot(self):
        """Register a robot with the server."""
        response = requests.post(f"{self.api_url}/add_robot", json={
                "device_address": self.device_address,
                "characteristic_uuid": self.characteristic_uuid,
        })
        return response.json()

    def connect(self):
        """Synchronous wrapper for the connect API."""
        response = requests.post(f"{self.api_url}/connect", json={
                "device_address": self.device_address,
                "characteristic_uuid": self.characteristic_uuid,
        })
        return response.json()

    def disconnect(self):
        """Synchronous wrapper for the disconnect API."""
        response = requests.post(f"{self.api_url}/disconnect", json={
                "device_address": self.device_address,
                "characteristic_uuid": self.characteristic_uuid,
        })
        return response.json()

    def send_command(self, command: str, need_data: bool = False):
        """Synchronous wrapper for sending commands."""
        c = self.connect()
        print("CONNECTION", c)

        data = {"command": command, "need_data": need_data}
        response = requests.post(f"{self.api_url}/send_command", json={
            "robot_connection" : {
                "device_address": self.device_address,
                "characteristic_uuid": self.characteristic_uuid,
            },
            "command" : data
        })
        return response.json()

    def get_angle_data(self):
        """Get angle data from the robot."""
        return self.send_command("ANGLE+0", need_data=True)

    def get_distance_data(self):
        """Get distance data from the robot."""
        return self.send_command("ENCODER+2", need_data=True)

    def move(self, distance: float):
        """Send a move command."""
        print("MOOOOOOOOOVING")
        return self.send_command(f"MOVE+{distance}")

    def turn(self, angle_in_degrees: float):
        """Send a turn command."""
        return self.send_command(f"TURN+{angle_in_degrees}")

    def reset_angle_data(self):
        command = f"ANGLE+1"
        return self.send_command(command, need_data=False)
    
    def reset_distance_data(self):
        """Reset distance data."""
        return self.send_command("ENCODER+3")

# Usage example:

if __name__ == "__main__":
    import json 
    with open('devices.json', 'r') as f:
        dct = json.load(f)
    
    d = dct["devices"][0]
    robot = Robot(device_address=d['address'], characteristic_uuid=d['write_uuid'])

    # Register and connect the robot
    print(robot.add_robot())
    print(robot.connect())

    # Get angle data
    print(robot.get_angle_data())

    # Move the robot
    print(robot.move(5))

    # Turn the robot
    print(robot.turn(90))

    # Get distance data
    print(robot.get_distance_data())

    # Disconnect from the robot
    print(robot.disconnect())
