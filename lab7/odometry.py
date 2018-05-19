#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time

def sqrt(x):
	return x**(0.5)

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius():
	"""Returns the radius of the Cozmo robot's front wheel in millimeters."""
	# ####
	# TODO: Empirically determine the radius of the robot's front wheel using the
	# cozmo_drive_straight() function. You can write a separate script for doing 
	# experiments to determine the radius. This function should return the radius
	# in millimeters. Write a comment that explains how you determined it and any
	# computation you do as part of this function.
	# ####

	# To determine the front wheel radius, I first fiddled with the distance passed
	# to the cozmo_drive_straight function such that the front wheel would make 4
	# complete revolutions. I added a marker to the wheel so I could tell.
	# I then ran the program 3 times and  marked the start and end positions of cozmo
	# on my desk.
	# Cozmo moved 13 7/16" when the front wheel made 4 complete revolutions.
	# Wheel radius = arc length / rotation angle
	# 13 7/16" ~= ‪341.3125‬ mm
	# Wheel radius = ‪341.3125‬ mm / (4 * (2*pi))
	# Wheel radius ~= 13.58mm

	return 13.58

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""
	# ####
	# TODO: Empirically determine the distance between the wheels of the robot using
	# robot.drive_wheels() function. Write a comment that explains how you determined
	# it and any computation you do as part of this function.
	# ####

	# To determine the distance between wheels, I called the robot.drive_wheels function
	# with the left wheel given a speed 1.5x that of the right wheel (45mm/s, 30mm/s).
	# I attached markers to either side of Cozmo and allowed him to draw circles on pieces
	# of paper. After allowing Cozmo to drive in circles several times, I measured the
	# diameter of the inner and outer cicles and averaged them to determine the diameter
	# of the circle formed by the movement of Cozmo's center point.
	# (13 7/16" + 19") / 2 = 16.21875"
	# 16.21875" = 411.95625 mm
	# Given this center diameter and the speed ratio of the wheels, I computed the distance
	# between the wheels.
	# Let Ci = Circumference of the arc formed by the inner wheels
	# Let Co = Circumference of the arc formed by the outer wheels
	# Co = 1.5 * Ci
	# Circumference of a circle = 2*pi*radius
	# Let Rc = 411.95625 mm / 2 = 205.978125 mm
	# Let Ri = Radius of the arc formed by the inner wheels
	# Let Ro = Radius of the arc formed by the outer wheels
	# Let d = distance between the wheels
	# Ri = Rc - (0.5 * d)
	# Ro = Rc + (0.5 * d)
	# 2*pi*Ro = 1.5 * (2*pi*Ri)
	# Ro = 1.5 * Ri
	# Rc + (0.5 * d) = 1.5 * (Rc - (0.5 * d))
	# Rc + (0.5 * d) = (1.5 * Rc) - (0.75 * d)
	# 1.25 * d = 0.5 * Rc
	# d = 0.4 * Rc = 0.4 * 205.978125 mm = 82.39125 mm

	return 82.39125

def rotate_front_wheel(robot, angle_deg):
	"""Rotates the front wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""
	# ####
	# TODO: Implement this function.
	# ####

	dist = cozmo.util.degrees(angle_deg).radians * get_front_wheel_radius()
	cozmo_drive_straight(robot, dist, 50)

def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	# ####
	# TODO: Implement your version of a driving straight function using the
	# robot.drive_wheels() function.
	# ####

	if speed is 0.0:
		return

	# Per the default behavior of drive_wheels(), acceleration = speed
	acceleration = speed

	distanceToTargetSpeed = sqr(speed) / (2.0 * acceleration)

	if distanceToTargetSpeed >= dist:
		time = sqrt((2.0 * dist) / acceleration)
	else:
		# time to achieve max speed
		time = speed / acceleration
		remainingDistance = dist - distanceToTargetSpeed
		time += remainingDistance / speed


	cozmo.robot.

	pass

def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	# ####
	# TODO: Implement your version of a rotating in place function using the
	# robot.drive_wheels() function.
	# ####
	pass

def my_go_to_pose1(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# using the my_drive_straight and my_turn_in_place functions. This should
	# include a sequence of turning in place, moving straight, and then turning
	# again at the target to get to the desired rotation (Approach 1).
	# ####
	pass

def my_go_to_pose2(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# using the robot.drive_wheels() function to jointly move and rotate the 
	# robot to reduce distance between current and desired pose (Approach 2).
	# ####
	pass

def my_go_to_pose3(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# as fast as possible. You can experiment with the built-in Cozmo function
	# (cozmo_go_to_pose() above) to understand its strategy and do the same.
	# ####
	pass

async def test(robot: cozmo.robot.Robot):
	#cozmo_drive_straight(robot, 341, 50)
	#await robot.drive_wheels(45, 30)
	#time.sleep(600)


def run(robot: cozmo.robot.Robot):

	print("***** Front wheel radius: " + str(get_front_wheel_radius()))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))

	## Example tests of the functions

	cozmo_drive_straight(robot, 62, 50)
	cozmo_turn_in_place(robot, 60, 30)
	cozmo_go_to_pose(robot, 100, 100, 45)

	rotate_front_wheel(robot, 90)
	my_drive_straight(robot, 62, 50)
	my_turn_in_place(robot, 90, 30)

	my_go_to_pose1(robot, 100, 100, 45)
	my_go_to_pose2(robot, 100, 100, 45)
	my_go_to_pose3(robot, 100, 100, 45)


if __name__ == '__main__':

	#cozmo.run_program(run)
	cozmo.run_program(test)



