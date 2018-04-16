#!/usr/bin/env python3

# Adam Heaney
# CSE P 590 B - Robotics
# Lab 5 - Servoing and Finite State Machines

import asyncio
import sys

import cv2
import numpy as np

sys.path.insert(0, '../lab4')
import find_ball

import cozmo

import time

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


# Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)

# Define a decorator as a subclass of Annotator; displays the ball
class BallAnnotator(cozmo.annotate.Annotator):

    ball = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BallAnnotator.ball is not None:

            #double size of bounding box to match size of rendered image
            BallAnnotator.ball = np.multiply(BallAnnotator.ball,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BallAnnotator.ball[0]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[1]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[2]*2, BallAnnotator.ball[2]*2)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            box = cozmo.util.ImageBox(BallAnnotator.ball[0]-2,
                                      BallAnnotator.ball[1]-2,
                                      4, 4)
            cozmo.annotate.add_img_box_to_image(image, box, "yellow", text=None)

            BallAnnotator.ball = None

# Global state machine instance
stateMachine = None

# Environment parameters. (contains the dimensions of the area within which Cozmo will search)
class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height

class State:
    def __init__(self, robot, environment):
        self.robot = robot
        self.environment = environment

# Behavioral state wherin Cozmos drives around an area until a ball is detected.
class SearchForBallState(State):
    def __init__(self, robot, environment):
        self.MOVEMENT_X_INCREMENT = 50
        self.MOVEMENT_Y_INCREMENT = 90
        super().__init__(robot, environment)

    async def enter(self):
        print("Enter SearchForBallState")

        await self.robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE + cozmo.util.degrees(15.0)).wait_for_completed()

        # Assume cozmo is in the center of the space to search for the ball.
        self.positionX = self.environment.width * 0.5
        self.positionY = self.environment.height * 0.5
        self.lastTurnDirectionWasRight = True

    async def exit(self):
        print("Exit SearchForBallState")

    async def tick(self, camera_image, ball):
        if not ball is None:
            await stateMachine.changeState(stateMachine.approachBallState)
            return

        # search for the ball using a simple back and forth pattern around cozmo's starting point
        if self.positionX < self.environment.width:
            moveDistance = min(self.environment.width - self.positionX, self.MOVEMENT_X_INCREMENT)
            await self.robot.drive_straight(cozmo.util.distance_mm(moveDistance), cozmo.util.speed_mmps(50), False).wait_for_completed()
            self.positionX = self.positionX + self.MOVEMENT_X_INCREMENT
        elif self.positionY < self.environment.height:
            turnAngle = 90 if self.lastTurnDirectionWasRight else -90
            moveDistance = min(self.environment.height - self.positionY, self.MOVEMENT_Y_INCREMENT)
            await self.robot.turn_in_place(cozmo.util.degrees(turnAngle)).wait_for_completed()
            await self.robot.drive_straight(cozmo.util.distance_mm(self.MOVEMENT_Y_INCREMENT), cozmo.util.speed_mmps(50), False).wait_for_completed()
            await self.robot.turn_in_place(cozmo.util.degrees(turnAngle)).wait_for_completed()
            self.positionY = self.positionY + self.MOVEMENT_Y_INCREMENT
            self.positionX = 0
            self.lastTurnDirectionWasRight = not self.lastTurnDirectionWasRight
        else:
            turnAngle = -90 if self.lastTurnDirectionWasRight else 90
            await self.robot.turn_in_place(cozmo.util.degrees(turnAngle)).wait_for_completed()
            await self.robot.drive_straight(cozmo.util.distance_mm(self.MOVEMENT_Y_INCREMENT), cozmo.util.speed_mmps(50), False).wait_for_completed()
            await self.robot.turn_in_place(cozmo.util.degrees(turnAngle)).wait_for_completed()
            self.positionY = 0
            self.positionX = 0

# Behavior state where in Cozmo will approach a ball once it has been detected.
class ApproachBallState(State):
    async def enter(self):
        print("Enter ApproachBallState")

    async def exit(self):
        print("Exit ApproachBallState")

    async def tick(self, camera_image, ball):
        if ball is None:
            # If the ball is lost, return to the search state.
            await stateMachine.changeState(stateMachine.searchForBallState)
            return
        elif ball[2] > 100:
            # If the radius of the ball is large enough, transition to the hit ball state.
            await stateMachine.changeState(stateMachine.hitBallState)
            return

        centerOffsetX = ball[0] - (camera_image.shape[1] * 0.5)
        centerOffsetY = ball[1] - (camera_image.shape[0] * 0.5)

        # Turn Cozmo's head towards the ball.
        adjustHeadTask = None
        if centerOffsetY > 10:
            adjustHeadTask = self.robot.set_head_angle(self.robot.head_angle + cozmo.util.degrees(-5.0))
        elif centerOffsetY < 10:
            adjustHeadTask = self.robot.set_head_angle(self.robot.head_angle + cozmo.util.degrees(5.0))

        # Turn towards the ball.
        if centerOffsetX > 10.0:
            await self.robot.turn_in_place(cozmo.util.degrees(-5), True).wait_for_completed()
        elif centerOffsetX < 10.0:
            await self.robot.turn_in_place(cozmo.util.degrees(5), True).wait_for_completed()

        if adjustHeadTask is not None:
            await adjustHeadTask.wait_for_completed()

        # Drive forwards.
        await self.robot.drive_straight(cozmo.util.distance_mm(20), cozmo.util.speed_mmps(50), False).wait_for_completed()

# Behavioral state where Cozmo hits the ball in front of him.
class HitBallState(State):
    async def enter(self):
        print("Enter HitBallState")

    async def exit(self):
        print("Exit HitBallState")

    async def tick(self, camera_image, ball):
        # Hit the ball.
        await self.robot.play_anim_trigger(cozmo.anim.Triggers.PouncePounce).wait_for_completed()

        # After 5 seconds, turn around and return to the search state.
        await asyncio.sleep(5)

        await self.robot.turn_in_place(cozmo.util.degrees(180)).wait_for_completed()
        await stateMachine.changeState(stateMachine.searchForBallState)

class StateMachine:
    def __init__(self, robot: cozmo.robot.Robot, environment: Environment):
        self.searchForBallState = SearchForBallState(robot, environment)
        self.approachBallState = ApproachBallState(robot, environment)
        self.hitBallState = HitBallState(robot, environment)
        self.currentState = None

    # Transition from one behavioral state to the next.
    async def changeState(self, newState):
        if not self.currentState is None:
            await self.currentState.exit()
        self.currentState = newState
        if not self.currentState is None:
            await self.currentState.enter()

async def run(robot: cozmo.robot.Robot):
    global stateMachine

    '''The run method runs once the Cozmo SDK is connected.'''

    #add annotators for battery level and ball bounding box
    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)

    # set initial state
    cozmoEnv = Environment(350, 350)
    stateMachine = StateMachine(robot, cozmoEnv)
    await stateMachine.changeState(stateMachine.searchForBallState)

    try:

        while True:
            #get camera image
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)

            #find the ball
            ball = find_ball.find_ball(opencv_image)

            #set annotator ball
            BallAnnotator.ball = ball

            # Update the current state.
            await stateMachine.currentState.tick(opencv_image, ball)

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)

