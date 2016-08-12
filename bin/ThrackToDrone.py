#!/usr/bin/python

# ThrackToDrone Research Copter Control Client
# 
# This script will call ThrackerZodd and calculate the velocites to allow the quad to follow an LED Marker
# Authors: Kiran Shila and Alejandro Robles

# Mavlink Velocity Definition
from pymavlink import mavutil 
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
	    0,       # time_boot_ms (not used)
	    0, 0,    # target system, target component
	    mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
	    0b0000111111000111, # type_mask (only speeds enabled)
	    0, 0, 0, # x, y, z positions (not used)
	    velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
	    0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
	    0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	vehicle.send_mavlink(msg)

# Create PID Object
from PID import PID
pid_x = PID(-1,0,0)
pid_y = PID(-1,0,0)
pid_z = PID(-1,0,0)
pid_yaw = PID(-1,0,0)
from subprocess import call
import subprocess
import fileinput, sys, os, time
call(["clear"])

# For Simulated Drone
print "Starting Simulator"
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# For Real Drone
#print "Connecting to Drone"
#connection_string = "/dev/cu.usbmodem1"

# Import DroneKit-Python
from dronekit import connect, VehicleMode

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# Get some vehicle attributes (state)
print "Get some vehicle attribute values:"
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name    # settable

# Wait for manual switch to GUIDED mode
while not vehicle.mode.name=='GUIDED' and not vehicle.armed:
	print " Waiting for user switch to GUIDED mode"
	vehicle.mode = VehicleMode("GUIDED")
	time.sleep(1)
print "Beginning LED Tracking"

# Open Tracking Software
thracker = subprocess.Popen("./ThrackerZodd",stdout=subprocess.PIPE, shell=False)

# Begin Tracking Loop
while True:
	with thracker.stdout:
		for line in iter(thracker.stdout.readline,b''):
			if vehicle.mode.name is not "GUIDED":
				print "User has changed flight modes - aborting LED Tracking"
				done = True
				break
			numbers_float = map(float, line.split())
			conversionFactor = 100
			numbers_meters = [x / conversionFactor for x in numbers_float]
			#print numbers_float
			if (numbers_float[0] == 0 and numbers_float[1] == 0 and numbers_float[2] == 0):
			 	print "No Marker Found."
			 	send_ned_velocity(0,0,0)
			else:
				print "Hey look, a marker"
				x_vel = pid_x.GenOut(numbers_meters[0])
				y_vel = pid_y.GenOut(numbers_meters[1])
				z_vel = pid_z.GenOut(numbers_meters[2] - 1)
				print "X Velocity is %f" % x_vel
				print "Y Velocity is %f" % y_vel
				print "Z Velocity is %f" % z_vel
				send_ned_velocity(x_vel,y_vel,z_vel)
		if done is True:
			break;

print "Quiting LED Tracking, switching to loiter."

vehicle.mode = VehicleMode("LOITER")
# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
sitl.stop()
print("Completed")