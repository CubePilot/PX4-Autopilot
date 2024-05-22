#!/usr/bin/env python

from pymavlink import mavutil
import time
import struct

connection_string = "/dev/ttyACM0"
baud_rate = 115200

vehicle = mavutil.mavlink_connection(connection_string, baud=baud_rate)

# Send heartbeat to make PX4 start talking mavlink
vehicle.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
)
time.sleep(1)

# Wait for the heartbeat message to find the system ID
vehicle.wait_heartbeat()

# Get the system and component IDs
system_id = vehicle.target_system
component_id = vehicle.target_component


def int_to_float(integer):
    # Pack the integer as a 32-bit unsigned integer
    packed_int = struct.pack("I", integer)

    # Unpack the packed integer as a float
    float_value = struct.unpack("f", packed_int)[0]

    return float_value


# Function to set a parameter
def set_param_int(param_id, value):
    # Create the PARAM_SET message
    param_set_msg = vehicle.mav.param_set_encode(
        system_id,  # target_system
        component_id,  # target_component
        param_id.encode("ascii"),  # param_id
        int_to_float(value),  # param_value
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,  # param_type
    )

    # Send the PARAM_SET message
    vehicle.mav.send(param_set_msg)

    # Wait for an acknowledgment from the vehicle
    while True:
        message = vehicle.recv_match(type="PARAM_VALUE", blocking=True)
        if message.param_id == param_id:
            print(f"Parameter {param_id} set to {value}")
            break


# Set the SYS_BL_UPDATE parameter to 1
set_param_int("SYS_BL_UPDATE", 1)

# Give some time for the parameter to be set
time.sleep(2)

# Send the reboot command
vehicle.mav.command_long_send(
    system_id,
    component_id,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # Command to reboot the system
    0,  # Confirmation
    1,  # Param 1 (1 for reboot)
    0,
    0,
    0,
    0,
    0,
    0,  # Unused parameters
)

print("Reboot command sent")

# Close the connection
vehicle.close()
