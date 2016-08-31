#!/usr/bin/python

# ThrackToDrone Research Copter Control Client
# This script will call ThrackerZodd and calculate the
# velocites to allow the quad to follow an LED Marker
# Authors: Kiran Shila and Alejandro Robles

# Mavlink Velocity Definition
from pymavlink import mavutil
from PID import PID
from subprocess import call
import subprocess
import signal
import os
# If simulated vehicle
# import dronekit_sitl
from dronekit import connect, VehicleMode


# Positions are relative to the current vehicle position
# in a frame based on the vehicle's current heading. Use
# this to specify a position x metres forward from
# the current vehicle position, y metres to the right, and
# z metres down (forward,right and down are "positive" values).


def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Relative to frame heading
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)

# Create PID Object
pid_x = PID(-1, 0, 0)
pid_y = PID(-1, 0, 0)
pid_z = PID(-1, 0, 0)
pid_yaw = PID(-1, 0, 0)

call(["clear"])

# For Simulated Drone
# print "Starting Simulator"
# sitl = dronekit_sitl.start_default()
# connection_string = sitl.connection_string()

# For Real Drone
print "Connecting to Drone"
connection_string = "/dev/ttyUSB0"

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True, baud=1500000)

# Get some vehicle attributes (state)
# print "Get some vehicle attribute values:"
# print " GPS: %s" % vehicle.gps_0
# print " Battery: %s" % vehicle.battery
# print " Last Heartbeat: %s" % vehicle.last_heartbeat
# print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name    # settable

# Get all channel values from RC transmitter
# print "Channel values from RC Tx:", vehicle.channels
printOnce = True
while vehicle.channels['6'] < 1500:
    if printOnce:
        printOnce = False
        print "Waiting for user to hit tracking switch"

print "Beginning LED Tracking"
vehicle.mode.name = 'GUIDED'

# Open Tracking Software
thracker = subprocess.Popen("./ThrackerZodd",
                            stdout=subprocess.PIPE, shell=False)

# Begin Tracking Loop
noMarkerPrint = False
# trackingDone = False
while True:
    if vehicle.channels['6'] < 1500:
        print "Waiting for user to hit tracking switch"
        continue
    else:
        # Read each line from ThrackerZodd
        with thracker.stdout:
            for line in iter(thracker.stdout.readline, b''):
                # This code runs FOR EACH line recieved from TZ
                # If the user either switches back the tracking switch
                # or changes the mode, double break
                if vehicle.channels['6'] < 1500 or vehicle.mode.name != 'GUIDED':
                        print "Halting LED Tracking, switching to loiter."
                        # Loiter failsafe
                        vehicle.mode.name = 'LOITER'
                        break
                # Cast read line into float array, divide all by 100 to get meters
                numbers_float = [float(num) for num in line.split()]
                conversionFactor = 100
                numbers_meters = [x / conversionFactor for x in numbers_float]
                # If all zero, no marker was found. Print that once
                if (numbers_float[0] == 0 and numbers_float[1] == 0 and numbers_float[2] == 0):
                    if not noMarkerPrint:
                        noMarkerPrint = True
                        print "No Marker Found."
                    send_ned_velocity(0, 0, 0)
                else:
                    # Put the positions in meters into PID generator
                    # Get velocities to center quad at 0,0,
                    noMarkerPrint = False
                    x_vel = pid_x.GenOut(numbers_meters[2] - 1)
                    y_vel = pid_y.GenOut(numbers_meters[0])
                    z_vel = pid_z.GenOut(numbers_meters[1])
                    print "X Velocity is %f" % x_vel
                    print "Y Velocity is %f" % y_vel
                    print "Z Velocity is %f" % z_vel
                    send_ned_velocity(x_vel, y_vel, z_vel)
                    
# Interrupt Tracking Subprocess
os.kill(thracker.pid, signal.SIGQUIT)

# Close vehicle object before exiting script
print "Tracking Complete"
vehicle.close()
# sitl.complete()
