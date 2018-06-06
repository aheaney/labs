#!/usr/bin/env python3

# Adam Heaney
# CSEP 590 Robotics - Lab 7

'''
This is starter code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, radians, Angle, Pose, distance_mm, speed_mmps
import math
import time
import sys

from odometry import cozmo_go_to_pose, my_go_to_pose1, my_go_to_pose2, my_go_to_pose3
sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose

class Vector2:
	def __init__(self, x, y):
		self.x = x
		self.y = y
	def add(self, v):
		return Vector2(
			self.x + v.x,
			self.y + v.y)
	def rotated(self, angle):
		return Vector2(
			(self.x * math.cos(angle)) + (self.y * -math.sin(angle)),
			(self.x * math.sin(angle)) + (self.y * math.cos(angle)))

# compute angle between (-pi,pi]
def shiftAngle(angle):
	angleMod = abs(angle) % math.tau
	if angle > 0.0:
		if angleMod <= math.pi:
			return angleMod
		else:
			return angleMod - math.tau
	else:
		if angleMod < math.pi:
			return -angleMod
		else:
			return (-angleMod) + math.tau

def move_relative_to_cube(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, when a cube is detected it 
	moves the robot to a given pose relative to the detected cube pose.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while cube is None:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Found a cube, pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")

	desired_pose_relative_to_cube = Pose(0, 100, 0, angle_z=degrees(90))

	# ####
	# TODO: Make the robot move to the given desired_pose_relative_to_cube.
	# Use the get_relative_pose function your implemented to determine the
	# desired robot pose relative to the robot's current pose and then use
	# one of the go_to_pose functions you implemented in Lab 6.
	# ####

	cubePosition = Vector2(cube.pose.position.x, cube.pose.position.y)
	cubeRotation = cube.pose.rotation.angle_z.radians
	desiredPositionCube = Vector2(desired_pose_relative_to_cube.position.x, desired_pose_relative_to_cube.position.y)
	desiredRotationCube = desired_pose_relative_to_cube.rotation.angle_z.radians

	desiredPositionWorld = desiredPositionCube.rotated(cubeRotation).add(cubePosition)
	desiredRotationWorld = shiftAngle(cubeRotation + desiredRotationCube)
	desiredPoseWorld = cozmo.util.pose_z_angle(desiredPositionWorld.x, desiredPositionWorld.y, 0.0, radians(desiredRotationWorld))

	desiredPoseRobot = get_relative_pose(desiredPoseWorld, robot.pose)
	my_go_to_pose1(robot, desiredPoseRobot.position.x, desiredPoseRobot.position.y, desiredPoseRobot.rotation.angle_z.degrees)

if __name__ == '__main__':

	cozmo.run_program(move_relative_to_cube)
