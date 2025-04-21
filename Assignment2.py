#!/usr/bin/env python3


import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed, PositionNedYaw, AccelerationNed, Attitude)
from drone_pid import Position_PID
import time


import rospy
import gazebo_msgs.msg
import geometry_msgs.msg
import sys

from utils import GPSGeometry




current_position = None
async def print_position(drone):
	global current_position
	async for position in drone.telemetry.position():
		current_position = position
		# print('update')
		


def callback(data):
	x = data.pose[2].position.x
	y = data.pose[2].position.y
	z = data.pose[2].position.z
	# print((x, y, z))
	global current_position
	current_position = (x, y, z)
	print(current_position)

def pose_listener():
	
	rospy.init_node('poseListener', anonymous=True)
	rospy.Subscriber('/gazebo/model_states/', gazebo_msgs.msg.ModelStates, callback)
	rospy.spin()


async def run():
	""" Does Offboard control using velocity body coordinates. """
	rospy.init_node('poseListener', anonymous=True)

	drone = System()
	await drone.connect(system_address="udp://:14540")

	asyncio.ensure_future(print_position(drone))

	print("Waiting for drone to connect...")
	async for state in drone.core.connection_state():
		if state.is_connected:
			print(f"-- Connected to drone!")
			break

	print("Waiting for drone to have a global position estimate...")
	async for health in drone.telemetry.health():
		if health.is_global_position_ok and health.is_home_position_ok:
			print("-- Global position estimate OK")
			break

	print("-- Arming")
	await drone.action.arm()

	print("-- Setting initial setpoint")
	await drone.offboard.set_position_ned(
		PositionNedYaw(0.0, 0.0, 0.0, 0.0))

 

	print("-- Starting offboard")
	try:
		await drone.offboard.start()
	except OffboardError as error:
		print(f"Starting offboard mode failed with error code: \
			  {error._result.result}")
		print("-- Disarming")
		await drone.action.disarm()
		return


	geo = GPSGeometry()

	await asyncio.sleep(6)


	global current_position
	origin_gps = (current_position.latitude_deg, current_position.longitude_deg)


	########################################################################
	# You are required to adjust the following PID parameters
	# Kp_p is the proportional gain for position control
	# Kp_v is the proportional gain for velocity control
	# Ki_v is the integral gain for velocity control
	# Kd_v is the derivative gain for velocity control
	########################################################################

	#Kp_p = (0.02, 0.05, 0.2)


	#Kp_v = (1.0, 0.8, 3)
	#Ki_v = (0.07, 0.09, 0.1)
	#Kd_v = (2.0, 2.0, 0.7)
	Kp_p = (0.05, 0.05, 0.2)


	Kp_v = (1, 1, 3)
	Ki_v = (0.5, 0.5, 0.1)
	Kd_v = (2,1.8, 0.6)
	########################################################################
	# End TODO
	########################################################################


	########################################################################
	# If executing takeoff task, set takeoff_task to True. Otherwise False
	########################################################################
	takeoff_task = False

	if takeoff_task:
		goal = (0.0, 0.0, 10.0)
	else:
		goal = (10.0, 5.0, 10.0)

	########################################################################
	# End TODO
	########################################################################


	t = range(200)

	posPID = Position_PID(Kp_p, 0.11)
	posPID.set_velocity_gain(Kp_v, Kd_v, Kd_v)
	posPID.set_velocity_limit(5, 5, 5)
	posPID.set_state((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0, (0.0, 0.0, 0.0))
	posPID.position_setpoint(goal)

	previous_time = time.time()
	dt = 0.11
	for i in t:
		posPID.position_control()
		posPID.velocity_control()

		current_gps = (current_position.latitude_deg, current_position.longitude_deg)
		north, east = geo.gps2NEmeter(origin_gps, current_gps)

		current_position_3d = (north, east, current_position.relative_altitude_m)
		print('North: ', current_position_3d[0], 'm East: ', current_position_3d[1], 'm Altitude: ', current_position_3d[2], 'm')



		
		pos, acc = posPID.calc_next_position(current_position_3d, dt)



		await drone.offboard.set_acceleration_ned(
			AccelerationNed(acc[0], acc[1], -acc[2]))
		await asyncio.sleep(0.1)

		current_time = time.time()
		dt = current_time - previous_time
		previous_time = current_time


	print("-- Stopping offboard")
	try:
		await drone.offboard.stop()
	except OffboardError as error:
		print(f"Stopping offboard mode failed with error code: \
			  {error._result.result}")


if __name__ == "__main__":

	asyncio.run(run())
	
