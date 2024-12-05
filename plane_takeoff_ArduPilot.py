import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# =========================
# Configuration Parameters
# =========================
CONNECTION_STRING = '127.0.0.1:14540'  # Update as necessary
TAKEOFF = 50  # Target takeoff altitude in meters

# =========================
# Connect to the Vehicle
# =========================
print("Connecting to vehicle...")
vehicle = connect(CONNECTION_STRING, wait_ready=True)

# =========================
# Utility Functions
# =========================

def arm_and_takeoff(target_altitude):
    print("Performing pre-arm checks...")
    while not vehicle.is_armable:
        print(f" Waiting for vehicle to become armable... Armable: {vehicle.is_armable}, EKF OK: {vehicle.ekf_ok}")
        time.sleep(2)

    print("Changing mode to GUIDED...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print(" Waiting for mode change to GUIDED...")
        time.sleep(1)

    print("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(2)

    print("Motors armed. Taking off...")
    vehicle.simple_takeoff(target_altitude)

    # Monitor altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {altitude:.2f} meters")
        if altitude >= target_altitude * 0.95:
            print("Takeoff successful!")
            break
        time.sleep(1)

# =========================
# Main Takeoff Sequence
# =========================

def main():
    # Your code logic here
    arm_and_takeoff(TAKEOFF)
    print("Test2 running...")
