#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy
import time
from cozmo.util import degrees

def sqr(x):
	return x * x

class Vector3:
	@staticmethod
	def fromCozmoPosition(cozmoPosition: cozmo.util.Position):
		return Vector3(cozmoPosition.x, cozmoPosition.y, cozmoPosition.z)

	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z
	def add(self, v):
		return Vector3(self.x + v.x, self.y + v.y, self.z + v.z)
	def subtract(self, v):
		return Vector3(self.x - v.x, self.y - v.y, self.z - v.z)
	def multiply(self, s):
		return Vector3(self.x * s, self.y * s, self.z * s)
	def dot(self, v):
		return (self.x * v.x) + (self.y * v.y) + (self.z * v.z)
	# https://betterexplained.com/articles/cross-product/
	def cross(self, v):
		return Vector3(
			(self.y * v.z) - (self.z * v.y),
			(self.z * v.x) - (self.x * v.z),
			(self.x * v.y) - (self.y * v.x))
	def magnitude(self):
		return (sqr(self.x) + sqr(self.y) + sqr(self.z))**(0.5)

class Vector4:
	def __init__(self, x, y, z, w):
		self.x = x
		self.y = y
		self.z = z
		self.w = w
	def magnitude(self):
		return (sqr(self.x) + sqr(self.y) + sqr(self.z) + sqr(self.w))**(0.5)
	def normalize(self):
		magnitude = self.magnitude()
		if magnitude > 0.0:
			self.x /= magnitude
			self.y /= magnitude
			self.z /= magnitude
			self.w /= magnitude

# http://www.utdallas.edu/~sxb027100/dock/quaternion.html
class Quaternion(Vector4):
	@staticmethod
	def fromCozmoQuaternion(cozmoQuaternion: cozmo.util.Quaternion):
		return Quaternion(cozmoQuaternion.q1, cozmoQuaternion.q2, cozmoQuaternion.q3, cozmoQuaternion.q0)

	def __init__(self, x, y, z, w):
		Vector4.__init__(self, x, y, z, w)
	def scalar(self):
		return self.w
	def vector(self):
		return Vector3(self.x, self.y, self.z)
	def inverse(self):
		return Quaternion(-self.x, -self.y, -self.z, self.w)
	def multiply(self, q: 'Quaternion'):
		scalar = (self.scalar() * q.scalar()) - self.vector().dot(q.vector())
		vector = q.vector().multiply(self.scalar()).add(
			self.vector().multiply(q.scalar()).add(
				self.vector().cross(q.vector())))
		return Quaternion(vector.x, vector.y, vector.z, scalar)
	def rotatePoint(self, point: Vector3):
		qPoint = Quaternion(point.x, point.y, point.z, 0.0)
		qPointRotated = self.multiply(qPoint.multiply(self.inverse()))
		return Vector3(qPointRotated.x, qPointRotated.y, qPointRotated.z)

v1 = Vector3(6, 7, 8)
v2 = Vector3(1, 2, 8)

v3 = v1.cross(v2)
print("Result ", v3.x, ", ", v3.y, ", ", v3.z)

q1 = Quaternion(2, 3, 4, 1)
q2 = Quaternion(6, 7, 0, 5)

q3 = q1.multiply(q2)
print("Result ", q3.x, ", ", q3.y, ", ", q3.z, ", ", q3.w)

q1.normalize()
vr = q1.rotatePoint(v1)
print("Result ", vr.x, ", ", vr.y, ", ", vr.z, " mv1 ", v1.magnitude(), " mvr ", vr.magnitude())

def get_relative_pose(object_pose: cozmo.util.Pose, reference_frame_pose: cozmo.util.Pose):
	# ####
	# TODO: Implement computation of the relative frame using numpy.
	# Try to derive the equations yourself and verify by looking at
	# the books or slides before implementing.
	# ####

	objectPosition = Vector3.fromCozmoPosition(object_pose.position)
	objectRotation = Quaternion.fromCozmoQuaternion(object_pose.rotation)
	objectRotation.normalize()

	referenceFramePosition = Vector3.fromCozmoPosition(reference_frame_pose.position)
	referenceFrameRotation = Quaternion.fromCozmoQuaternion(reference_frame_pose.rotation)
	referenceFrameRotation.normalize()

	positionDeltaWorldSpace = objectPosition.subtract(referenceFramePosition)
	positionDeltaReferenceSpace = referenceFrameRotation.inverse().rotatePoint(positionDeltaWorldSpace)

	rotationDelta = objectRotation.multiply(referenceFrameRotation.inverse())

	return cozmo.util.pose_quaternion(
		positionDeltaReferenceSpace.x, positionDeltaReferenceSpace.y, positionDeltaReferenceSpace.z,
		rotationDelta.w, rotationDelta.x, rotationDelta.y, rotationDelta.z)

def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				#print("Cube pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
				print("Cube p: %s" % get_relative_pose(cube.pose, robot.pose))
				print("Actual: %s" % robot.pose.define_pose_relative_this(cube.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")
		time.sleep(1)


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
