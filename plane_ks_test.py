import time
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil

# =========================
# Configuration Parameters
# =========================
CONNECTION_STRING = '127.0.0.1:14540' 
TAKEOFF = 100 

# Airspeed settings (in m/s)
AIRSPEED_MAX = 18  # Maximum airspeed 
AIRSPEED_MIN = 5   # Minimum airspeed 
AIRSPEED_CRUISE = 9  # Cruise airspeed 

# Throttle settings
THR_MAX = 40  # Maximum throttle percentage 
TRIM_THROTTLE = 25  # Throttle percentage for level flight

# Pitch angle settings
PTCH_LIM_MAX_DEG = 20  # Maximum pitch angle

# =========================
# Connect to the Vehicle
# =========================
print("Connecting to vehicle...")
vehicle = connect(CONNECTION_STRING, wait_ready=True)

# =========================
# Set Flight Parameters
# =========================
def set_flight_parameters():
    print("Setting flight parameters...")

    # Set throttle max (THR_MAX)
    vehicle.parameters['THR_MAX'] = THR_MAX
    print(f" THR_MAX set to {THR_MAX}%")

    # Set throttle trim (TRIM_THROTTLE)
    vehicle.parameters['TRIM_THROTTLE'] = TRIM_THROTTLE
    print(f" TRIM_THROTTLE set to {TRIM_THROTTLE}%")

    # Set airspeed limits
    vehicle.parameters['AIRSPEED_MAX'] = AIRSPEED_MAX
    vehicle.parameters['AIRSPEED_MIN'] = AIRSPEED_MIN
    vehicle.parameters['AIRSPEED_CRUISE'] = AIRSPEED_CRUISE 
    print(f" AIRSPEED_MAX set to {AIRSPEED_MAX}")
    print(f" AIRSPEED_MIN set to {AIRSPEED_MIN}")
    print(f" AIRSPEED_CRUISE set to {AIRSPEED_CRUISE } m/s")

    # Set maximum pitch angle (PTCH_LIM_MAX_DEG)
    vehicle.parameters['PTCH_LIM_MAX_DEG'] = PTCH_LIM_MAX_DEG
    print(f" PTCH_LIM_MAX_DEG set to {PTCH_LIM_MAX_DEG} degrees")

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

    # Monitor altitude and set airspeed after reaching target altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {altitude:.2f} meters")
        
        # When the vehicle reaches 100m altitude, set airspeed to AIRSPEED_CRUISE
        if altitude >= target_altitude * 0.95:
            print("Target altitude reached. Setting airspeed to AIRSPEED_CRUISE...")
            vehicle.airspeed = AIRSPEED_CRUISE  # Set to cruise airspeed
            print(f" Airspeed set to {AIRSPEED_CRUISE} m/s")

        # Ensure airspeed does not exceed max or min limits
        if vehicle.airspeed > AIRSPEED_MAX:
            vehicle.airspeed = AIRSPEED_MAX
            print(f" Airspeed limited to {AIRSPEED_MAX} m/s")
        elif vehicle.airspeed < AIRSPEED_MIN:
            vehicle.airspeed = AIRSPEED_MIN
            print(f" Airspeed limited to {AIRSPEED_MIN} m/s")
        
        # Exit condition: check if the altitude is close enough to the target
        if altitude >= target_altitude:
            print("Takeoff successful!")
            break
        time.sleep(1)

'''def set_altitude():
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # Target system, target component
        mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,  # Command
        0,  # Confirmation
        2,  # Descent rate (m/s), 0 for default
        70,  # Target altitude
        0, 0, 0, 0, 0    # Unused parameters
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    print(f"Changing altitude to 70.")'''


def upload_mission():  
    cmds = vehicle.commands
    cmds.clear()  # Clear existing mission

    cmds.add(Command(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
        0,1,
        2,0,0,0,
        0,
        0,
        70,
    ))
    
    # Add a waypoint
    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1,
        0, 0, 0, 0,
        -35.343261,  # Target latitude
        149.165230,  # Target longitude
        70  # Altitude
    ))

    cmds.add(Command(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 1,
        0, 0, 0, 0,
        -35.543261,  # Target latitude
        149.165230,  # Target longitude
        70  # Altitude
    ))
    
    cmds.upload()
    print("Mission uploaded successfully.")


def execute_mission():
    print("Starting mission...")
    
    # Attempt to change mode to AUTO
    vehicle.mode = VehicleMode("AUTO")
    timeout = 15  # Wait for up to 15 seconds for mode change
    start_time = time.time()
    
    while vehicle.mode.name != "AUTO":
        print(" Waiting for mode change to AUTO...")
        time.sleep(1)
        
        # Exit if timeout occurs
        if time.time() - start_time > timeout:
            print("ERROR: Mode change to AUTO failed. Check mission and vehicle state.")
            return

    print("Mission started in AUTO mode.")


# =========================
# Main Takeoff Sequence
# =========================
def main():
    set_flight_parameters()
    arm_and_takeoff(TAKEOFF)
    upload_mission()
    execute_mission()
    print("Test2 running...")

if __name__ == "__main__":
    main()
