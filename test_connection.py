import asyncio
import json
from bleak import BleakScanner

async def find_robots():
    devices = []
    scanner = BleakScanner()
    
    await scanner.start()
    await asyncio.sleep(15.0)
    await scanner.stop()
    
    for d in scanner.discovered_devices:
        if "3piRobot" in d.name:
            devices.append({
                "name": d.name,
                "address": d.address,
                "write_uuid": "0000ffe1-0000-1000-8000-00805f9b34fb"
            })
    
    # Save to JSON
    with open("devices.json", "w") as f:
        json.dump({"devices": devices}, f, indent=4)
        
    print(f"Found {len(devices)} robots. Saved to devices.json")

if __name__ == "__main__":
    asyncio.run(find_robots())

        