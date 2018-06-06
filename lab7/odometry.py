#!/usr/bin/env python3

# Adam Heaney
# CSEP 590 Robotics - Lab 7

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time

def sqrt(x):
	return x**(0.5)

def sqr(x):
	return x * x

def sign(x):
	if x < 0.0:
		return -1.0
	else:
		return 1.0

def clampAngle(angle):
    angleMod = abs(angle) % 360.0
    if angle > 0.0:
        if angleMod <= 180.0:
            return angleMod
        else:
            return angleMod - 360.0
    else:
        if angleMod < 180.0:
            return -angleMod
        else:
            return (-angleMod) + 360.0

class Vector2:
	def __init__(self, x, y):
		self.x = x
		self.y = y
	def distanceTo(self, v):
		return self.subtract(v).magnitude()
	def add(self, v):
		return Vector2(self.x + v.x, self.y + v.y)
	def subtract(self, v):
		return Vector2(self.x - v.x, self.y - v.y)
	def magnitude(self):
		return sqrt(sqr(self.x) + sqr(self.y))
	def rotated(self, angle):
		return Vector2(
			(self.x * math.cos(angle)) + (self.y * -math.sin(angle)),
			(self.x * math.sin(angle)) + (self.y * math.cos(angle)))

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

COMPLETE_MOVEMENT_DELAY = 0.3

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

	if speed == 0.0:
		return

	direction = sign(speed) * sign(dist)
	speed = abs(speed)
	dist = abs(dist)
	acceleration = speed # Per the default behavior of drive_wheels(), acceleration = speed

	distanceToTargetSpeed = sqr(speed) / (2.0 * acceleration)

	t = 0.0
	if distanceToTargetSpeed >= dist:
		t = sqrt((2.0 * dist) / acceleration)
	else:
		# time to achieve max speed
		t = speed / acceleration
		remainingDistance = dist - distanceToTargetSpeed
		t += remainingDistance / speed

	wheelSpeed = direction * speed
	robot.drive_wheels(wheelSpeed, wheelSpeed, acceleration, acceleration, t)
	time.sleep(COMPLETE_MOVEMENT_DELAY)

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

	if speed == 0.0:
		return

	direction = sign(speed) * sign(angle)
	speed = abs(speed)
	angle = abs(angle)

	wheelRadius = get_distance_between_wheels() * 0.5
	dist = cozmo.util.degrees(angle).radians * wheelRadius
	displacementspeed = cozmo.util.degrees(speed).radians * wheelRadius

	acceleration = displacementspeed # Per the default behavior of drive_wheels(), acceleration = speed

	distanceToTargetSpeed = sqr(displacementspeed) / (2.0 * acceleration)

	t = 0.0
	if distanceToTargetSpeed >= dist:
		t = sqrt((2.0 * dist) / acceleration)
	else:
		# time to achieve max speed
		t = displacementspeed / acceleration
		remainingDistance = dist - distanceToTargetSpeed
		t += remainingDistance / displacementspeed

	wheelSpeed = direction * displacementspeed
	robot.drive_wheels(-wheelSpeed, wheelSpeed, acceleration, acceleration, t)
	time.sleep(COMPLETE_MOVEMENT_DELAY)

ANGULAR_VELOCITY_DEGpS = 30.0
DISPLACEMENT_VELOCITY_MMpS = 40.0
DISTANCE_EPSILON = 0.5

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

	vTarget = Vector2(x, y)
	distance = vTarget.magnitude()
	targetHeading = 0

	if distance > DISTANCE_EPSILON:
		targetHeading = math.degrees(math.atan2(vTarget.y, vTarget.x))
		angle = clampAngle(targetHeading)
		my_turn_in_place(robot, angle, ANGULAR_VELOCITY_DEGpS)

		distance = vTarget.magnitude()
		my_drive_straight(robot, distance, DISPLACEMENT_VELOCITY_MMpS)

	angle = clampAngle(angle_z - targetHeading)
	my_turn_in_place(robot, angle, ANGULAR_VELOCITY_DEGpS)

DISTANCE_ALLOWANCE = 30.0

GAIN_P1 = 0.13
GAIN_P2 = 0.13
GAIN_P3 = 0.13

TIME_STEP = 0.5

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

	distanceBetweenWheels = get_distance_between_wheels()

	initialPosition = Vector2(robot.pose.position.x, robot.pose.position.y)
	initialRotation = robot.pose.rotation.angle_z.degrees
	
	goalPosition = Vector2(x, y)
	goalRotation = angle_z

	if goalPosition.magnitude() < DISTANCE_EPSILON:
		my_turn_in_place(robot, angle_z, ANGULAR_VELOCITY_DEGpS)
		return

	currentPosition = Vector2(0.0, 0.0)
	currentRotation = 0.0

	while currentPosition.distanceTo(goalPosition) > DISTANCE_ALLOWANCE:
		distance = currentPosition.distanceTo(goalPosition)
		headingdelta = clampAngle(currentRotation - math.degrees(math.atan2(currentPosition.y - goalPosition.y, currentPosition.x - goalPosition.x)))
		angledelta = clampAngle(goalRotation - currentRotation)

		displacementOffset = GAIN_P1 * distance
		angleOffset = (GAIN_P2 * math.radians(clampAngle(headingdelta))) + (GAIN_P3 * math.radians(clampAngle(angledelta)))

		velocityLeft = 0.5 * ((2.0 * displacementOffset) - (angleOffset * distanceBetweenWheels))
		velocityRight = 0.5 * ((2.0 * displacementOffset) + (angleOffset * distanceBetweenWheels))

		robot.drive_wheels(velocityLeft, velocityRight, None, None, TIME_STEP)

		robotPositionWorld = Vector2(robot.pose.position.x, robot.pose.position.y)
		robotRotationWorld = robot.pose.rotation.angle_z.degrees

		currentPosition = robotPositionWorld.subtract(initialPosition).rotated(-math.radians(initialRotation))
		currentRotation = clampAngle(robotRotationWorld - initialRotation)

# a bit less than 90 to ensure cozmo turns past 90 degrees
MIN_ANGLE_TO_TURN_IN_PLACE = 80.0

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

	# Just turn in place if the robot is near the goal.
	vTarget = Vector2(x, y)
	distance = vTarget.magnitude()
	if distance < DISTANCE_EPSILON:
		my_turn_in_place(robot, angle_z, ANGULAR_VELOCITY_DEGpS)
		return

	# Turn within 90 degrees of the goal if it is behind the robot
	targetHeading = math.degrees(math.atan2(vTarget.y, vTarget.x))
	angle = clampAngle(targetHeading)
	if abs(angle) > MIN_ANGLE_TO_TURN_IN_PLACE:
		my_turn_in_place(robot, angle - (sign(angle) * MIN_ANGLE_TO_TURN_IN_PLACE), ANGULAR_VELOCITY_DEGpS)

	# Advance towards the goal using my_go_to_pose2
	# Turn in place towards the goal rotation past 90 degrees 
	angleToFinalPose = clampAngle(angle_z - targetHeading)
	if abs(angleToFinalPose) > MIN_ANGLE_TO_TURN_IN_PLACE:
		angleDifference = angleToFinalPose - (sign(angleToFinalPose) * MIN_ANGLE_TO_TURN_IN_PLACE)
		my_go_to_pose2(robot, x, y, angle_z - angleDifference)
		my_turn_in_place(robot, angleDifference, ANGULAR_VELOCITY_DEGpS)
	else:
		my_go_to_pose2(robot, x, y, angle_z)

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

	cozmo.run_program(run)



