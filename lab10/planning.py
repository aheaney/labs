
#author1: Adam Heaney
#author2:

from grid import *
from visualizer import *
import threading
from queue import Queue, PriorityQueue
import math
import cozmo
import functools
import time


class Vector2:
    @staticmethod
    def fromCell(cell):
        return Vector2(cell[0], cell[1])
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def squaredDistanceTo(self, v):
        dx = self.x - v.x
        dy = self.y - v.y
        return (dx * dx) + (dy * dy)
    def roughlyEquals(self, v):
        return abs(self.x - v.x) < 0.01 and abs(self.y - v.y) < 0.01
    def distanceTo(self, v):
        return self.squaredDistanceTo(v)**0.5
    def length(self):
        return ((self.x * self.x) + (self.y * self.y))**0.5
    def normalized(self):
        myLength = self.length()
        if myLength is 0:
            return Vector2(1, 0)
        return Vector2(self.x / myLength, self.y / myLength)
    def rotated(self, angleRadians):
        return Vector2(
            (math.cos(angleRadians) * self.x) - (math.sin(angleRadians) * self.y),
            (math.sin(angleRadians) * self.x) + (math.cos(angleRadians) * self.y))
    def add(self, v):
        return Vector2(self.x + v.x, self.y + v.y)
    def subtract(self, v):
        return Vector2(self.x - v.x, self.y - v.y)
    def multiply(self, s):
        return Vector2(self.x * s, self.y * s)
    def dot(self, v):
        return (self.x * v.x) + (self.y * v.y)
    def projectOnTo(self, v):
        length = v.length()
        if length is 0:
            return 0
        return self.dot(v) / v.length()


def astar(grid, heuristic):
    """Perform the A* search algorithm on a defined grid

        Arguments:
        grid -- CozGrid instance to perform search on
        heuristic -- supplied heuristic function
    """

    class MapNode:
        def __init__(self, cell, cost, parent):
            self.cell = cell
            self.cost = cost
            self.parent = parent

    @functools.total_ordering
    class FrontierElement:
        def __init__(self, cell, cost, parent, estimatedCost):
            self.node = MapNode(cell, cost, parent)
            self.estimatedCost = estimatedCost
        def __lt__(self, other):
            return self.estimatedCost < other.estimatedCost
        def __eq__(self, other):
            return self.estimatedCost is other.estimatedCost

    frontier = PriorityQueue()
    visitedNodes = set()
    frontier.put(FrontierElement(grid.getStart(), 0, None, 0))

    path = []

    while not frontier.empty():
        currentElement = frontier.get()
        grid.addVisited(currentElement.node.cell)
        visitedNodes.add(currentElement.node.cell)

        if currentElement.node.cell in grid.getGoals():
            currentNode = currentElement.node
            while currentNode is not None:
                path.insert(0, currentNode.cell)
                currentNode = currentNode.parent
            break

        for neighbor in grid.getNeighbors(currentElement.node.cell):
            neighborCoord = neighbor[0]

            if neighborCoord in visitedNodes:
                continue

            neighborCost = neighbor[1]
            cheapestGoal = min(grid.getGoals(), key=lambda goal: Vector2.fromCell(neighborCoord).squaredDistanceTo(Vector2.fromCell(goal)))

            cost = currentElement.node.cost + neighborCost

            frontier.put(FrontierElement(neighborCoord, cost, currentElement.node, cost + heuristic(neighborCoord, cheapestGoal)))

    grid.setPath(path)


def heuristic(current, goal):
    """Heuristic function for A* algorithm

        Arguments:
        current -- current cell
        goal -- desired goal cell
    """

    return Vector2.fromCell(current).distanceTo(Vector2.fromCell(goal))

# Lab 10 Part 2 Implementation:

CUBE_SIDE_LENGTH_MM = 45
STOPPING_DISTANCE_TO_GOAL_MM = 30
COZMO_RADIUS_MM = 50

def clampToGrid(coord, grid: CozGrid):
    return (min(grid.width - 1, max(0, coord[0])), min(grid.height - 1, max(0, coord[1])))

class RobotMovementDirections:
    def __init__(self):
        self.goalKnown = False
        self.poseQueue = Queue()

class World:
    def __init__(self, origin: cozmo.util.Pose, gridStart: Vector2, gridScale):
        self.cubes = { } # guarded by self.lock
        self.lock = threading.Lock()
        self.goalKnown = False
        self.poseQueue = []
        self.movementDirectionsUpdatedEvent = threading.Event()
        self.reachedGoalEvent = threading.Event()
        self.originXY = Vector2(origin.position.x, origin.position.y)
        self.originR = origin.rotation.angle_z.radians
        self.gridStart = gridStart
        if gridScale is 0:
            self.gridScale = 1.0
        else:
            self.gridScale = gridScale
        self.invGridScale = 1.0 / self.gridScale
        self.goalDirection = None

    # Convert pose from cozmo world space to map space
    def transformCozmoWorldToMapWorld2d(self, pose: cozmo.util.Pose):
        positionCozmoWorld = Vector2(pose.position.x, pose.position.y)
        rotationCozmoWorld = pose.rotation.angle_z.radians
        pWorld = positionCozmoWorld.subtract(self.originXY).rotated(-self.originR).multiply(self.invGridScale).add(self.gridStart)
        rWorld = -self.originR + rotationCozmoWorld
        return cozmo.util.pose_z_angle(pWorld.x, pWorld.y, 0.0, angle_z=cozmo.util.radians(rWorld))

    # Convert pose from map space to cozmo world space
    def transformMapWorld2dToCozmoWorld(self, pose: cozmo.util.Pose):
        positionMap = Vector2(pose.position.x, pose.position.y)
        rotationMap = pose.rotation.angle_z.radians
        pWorld = positionMap.subtract(self.gridStart).multiply(self.gridScale).rotated(self.originR).add(self.originXY)
        rWorld = rotationMap + self.originR
        return cozmo.util.pose_z_angle(pWorld.x, pWorld.y, 0.0, angle_z=cozmo.util.radians(rWorld))

    # Submit sensed cube data
    def updateCubes(self, cubes):
        self.lock.acquire()
        for cube in cubes:
            self.cubes[cube.id] = cube
        self.lock.release()

    # Output current path information
    def latchMovementDirections(self):
        directions = RobotMovementDirections()
        self.lock.acquire()
        directions.goalKnown = self.goalKnown
        for pose in self.poseQueue:
            directions.poseQueue.put(pose)
        self.lock.release()
        return directions

    # Clear the grid and generate a new path based on currently available cube data
    def replan(self, robot: cozmo.robot.Robot, grid: CozGrid):
        self.lock.acquire()

        print("start replanning")

        grid.clearGoals()
        grid.clearObstacles()
        grid.clearPath()
        grid.clearVisited()
        grid.clearStart()

        self.goalKnown = False
        for cubeId, cube in self.cubes.items():
            cubePose = self.transformCozmoWorldToMapWorld2d(
                cozmo.util.pose_z_angle(cube.position.x, cube.position.y, 0.0, angle_z=cozmo.util.radians(cube.rotation)))
            cubePosition = Vector2(cubePose.position.x, cubePose.position.y)
            cubeRotation = cubePose.rotation.angle_z.radians
            sideLength = cube.sideLength * self.invGridScale
            cubeOBB = OrientedRect(cubePosition, cubeRotation, sideLength, sideLength)
            obstacleList = cubeOBB.renderToGridCoords(grid)

            # LightCube1Id is the goal cube
            if cubeId is cozmo.objects.LightCube1Id:
                goalDistance = (sideLength * 0.5) + (STOPPING_DISTANCE_TO_GOAL_MM * self.invGridScale)
                goalDir = Vector2(-1, 0).rotated(cubeOBB.rotation)
                goalPosition = goalDir.multiply(goalDistance).add(cubeOBB.position)
                self.goalDirection = goalDir.multiply(-1.0)
                goalCoord = clampToGrid((int(round(goalPosition.x)), int(round(goalPosition.y))), grid)
                grid.addGoal(goalCoord)
                self.goalKnown = True
                if goalCoord in obstacleList:
                    obstacleList.remove(goalCoord)

            grid.addObstacles(obstacleList)

        if not self.goalKnown:
            # set the grid center as the goal by default
            grid.addGoal(clampToGrid((int(round(grid.width * 0.5)), int(round(grid.height * 0.5))), grid))

        robotPoseMap = self.transformCozmoWorldToMapWorld2d(robot.pose)
        grid.setStart(clampToGrid((int(round(robotPoseMap.position.x)), int(round(robotPoseMap.position.y))), grid))

        astar(grid, heuristic)

        self.generatePoseQueueFromPath(grid.getPath())

        print("finish replanning")

        self.movementDirectionsUpdatedEvent.set()

        self.lock.release()

    # Coalesce path coordinates into a series of poses for cozmo to navigate to.
    def generatePoseQueueFromPath(self, path):
        self.poseQueue = []
        # Assume we're at the goal already if a degenerate path is given.
        if len(path) < 2:
            return

        lastDirection = Vector2.fromCell(path[1]).subtract(Vector2.fromCell(path[0])).normalized()
        i = 2
        while i < len(path):
            direction = Vector2.fromCell(path[i]).subtract(Vector2.fromCell(path[i - 1])).normalized()
            if not direction.roughlyEquals(lastDirection):
                heading = lastDirection.add(direction)
                self.poseQueue.append(cozmo.util.pose_z_angle(
                    path[i - 1][0], path[i - 1][1], 0.0,
                    angle_z=cozmo.util.radians(math.atan2(heading.y, heading.x))))
            lastDirection = direction
            i = i + 1

        # add the end cap
        finalHeading = lastDirection
        if self.goalKnown and self.goalDirection is not None:
            finalHeading = self.goalDirection

        self.poseQueue.append(cozmo.util.pose_z_angle(
            path[len(path) - 1][0], path[len(path) - 1][1], 0.0,
            angle_z=cozmo.util.radians(math.atan2(finalHeading.y, finalHeading.x))))

# sensed cube data
class Cube:
    def __init__(self, cozmoCube: cozmo.objects.LightCube, sideLength):
        self.id = cozmoCube.cube_id
        self.position = Vector2(cozmoCube.pose.position.x, cozmoCube.pose.position.y)
        self.rotation = cozmoCube.pose.rotation.angle_z.radians
        self.sideLength = sideLength
    def roughlyEquals(self, otherCube):
        return self.id is otherCube.id and \
            abs(self.position.x - otherCube.position.x) < 3.00 and \
            abs(self.position.y - otherCube.position.y) < 3.00 and \
            abs(self.rotation - otherCube.rotation) < 0.05

# axis-aligned 2d rect
class Rect:
    def __init__(self, left, right, top, bottom):
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom
        self.vertices = self.computeCornerVertices()
    def computeCornerVertices(self):
        return (Vector2(self.right, self.top), Vector2(self.right, self.bottom),
            Vector2(self.left, self.bottom), Vector2(self.left, self.top))

# oriented 2d rect
class OrientedRect:
    def __init__(self, position, rotation, width, height):
        self.position = position
        self.rotation = rotation
        self.width = width
        self.height = height
        self.vertices = self.computeCornerVertices()
        self.extents = self.extents()
    def computeCornerVertices(self):
        halfWidth = self.width * 0.5
        halfHeight = self.height * 0.5
        return (Vector2(halfWidth, halfHeight).rotated(self.rotation).add(self.position),
            Vector2(halfWidth, -halfHeight).rotated(self.rotation).add(self.position),
            Vector2(-halfWidth, -halfHeight).rotated(self.rotation).add(self.position),
            Vector2(-halfWidth, halfHeight).rotated(self.rotation).add(self.position))
    def extents(self):
        top = max(self.vertices[0].y, self.vertices[1].y, self.vertices[2].y, self.vertices[3].y)
        right = max(self.vertices[0].x, self.vertices[1].x, self.vertices[2].x, self.vertices[3].x)
        bottom = self.position.y - (top - self.position.y)
        left = self.position.x - (right - self.position.x)
        return Rect(left, right, top, bottom)
    def intersectsRect(self, rect: Rect):
        # check against rect axes
        if self.extents.right < rect.left or rect.right < self.extents.left or \
            self.extents.top < rect.bottom or rect.top < self.extents.bottom:
            return False

        # check against my axes
        vAxis1 = self.vertices[1].subtract(self.vertices[0]) # height axis
        projectionMin = None
        projectionMax = None
        for vertex in rect.vertices:
            projection = vertex.subtract(self.vertices[0]).projectOnTo(vAxis1)
            if projectionMin is None or projection < projectionMin:
                projectionMin = projection
            if projectionMax is None or projection > projectionMax:
                projectionMax = projection
        if projectionMax < 0.0 or projectionMin > self.height:
            return False

        vAxis2 = self.vertices[2].subtract(self.vertices[1]) # width axis
        projectionMin = None
        projectionMax = None
        for vertex in rect.vertices:
            projection = vertex.subtract(self.vertices[1]).projectOnTo(vAxis2)
            if projectionMin is None or projection < projectionMin:
                projectionMin = projection
            if projectionMax is None or projection > projectionMax:
                projectionMax = projection
        if projectionMax < 0.0 or projectionMin > self.width:
            return False

        return True

    # 'rasterize' this rect as a list of occupied grid locations
    def renderToGridCoords(self, grid: CozGrid):
        coords = []
        left = max(0, int(math.floor(self.extents.left)))
        y = max(0, int(math.floor(self.extents.bottom)))
        right = min(grid.width, int(math.ceil(self.extents.right)))
        top = min(grid.height, int(math.ceil(self.extents.top)))

        print("render cube")

        while y < top:
            # find left
            lhit = None
            rhit = None
            x = left
            while x < right and lhit is None:
                if self.intersectsRect(Rect(x, x + 1, y + 1, y)):
                    lhit = x
                x = x + 1
            if lhit is None:
                print("rc: no hit")
                y = y + 1
                continue
            x = right - 1
            while x > lhit and rhit is None:
                if self.intersectsRect(Rect(x, x + 1, y + 1, y)):
                    rhit = x
                x = x - 1
            print("rc: hit l ", lhit, " r ", rhit)
            if rhit is None:
                coord = (lhit, y)
                coords.append(coord)
            else:
                x = lhit
                while x <= rhit:
                    coord = (x, y)
                    coords.append(coord)
                    x = x + 1
            y = y + 1

        return coords

# robot movement thread processor, runs continuously while the robot navigates to the goal
# controls the movement behavior of the robot, based on what plan is currently available
def updateMovementThread(robot: cozmo.robot.Robot, world: World):
    global stopevent

    currentMovementAction: cozmo.action.Action = None
    currentMovementDirections: RobotMovementDirections = None

    while not stopevent.is_set() and not world.reachedGoalEvent.is_set():
        if world.movementDirectionsUpdatedEvent.is_set():
            world.movementDirectionsUpdatedEvent.clear()
            currentMovementDirections = world.latchMovementDirections()
            if currentMovementAction is not None and currentMovementAction.is_running:
                currentMovementAction.abort(log_abort_messages=True)
                currentMovementAction = None
        elif currentMovementAction is None or currentMovementAction.is_completed:
            if currentMovementDirections is not None:
                if not currentMovementDirections.poseQueue.empty():
                    currentTargetPose = currentMovementDirections.poseQueue.get()
                    currentTargetPoseWorld = world.transformMapWorld2dToCozmoWorld(currentTargetPose)
                    print('current pose: x ', str(robot.pose.position.x), ' y ', str(robot.pose.position.y), ' r ', str(robot.pose.rotation.angle_z.degrees), \
                        ' new target pose: x ', str(currentTargetPoseWorld.position.x), ' y ', str(currentTargetPoseWorld.position.y), ' r ', str(currentTargetPoseWorld.rotation.angle_z.degrees), \
                        ' ntp map relative: x ', str(currentTargetPose.position.x), ' y ', str(currentTargetPose.position.y), ' r ', str(currentTargetPose.rotation.angle_z.degrees))
                    currentMovementAction = robot.go_to_pose(currentTargetPoseWorld, in_parallel=True)
                elif not currentMovementDirections.goalKnown:
                    currentMovementAction = robot.turn_in_place(cozmo.util.degrees(45.0), in_parallel=True)
                else:
                    world.reachedGoalEvent.set()
        else:
            time.sleep(0.01) # yield

# robot mapping thread processor, runs continuously while the robot navigates to the goal
# submits updates to the map when new data is sensed by the robot
def updateMapThread(robot: cozmo.robot.Robot, world: World, grid: CozGrid):
    global stopevent

    cubeData = { }

    while not stopevent.is_set() and not world.reachedGoalEvent.is_set():
        observedCubes = robot.world.wait_until_observe_num_objects(num=3, object_type=cozmo.objects.LightCube, timeout=0.5)
        print("Observed ", len(observedCubes), " cubes")
        cubesToUpdate = []
        for observedCube in observedCubes:
            cube = Cube(observedCube, CUBE_SIDE_LENGTH_MM + COZMO_RADIUS_MM)
            if cube.id not in cubeData or not cubeData[cube.id].roughlyEquals(cube):
                cubesToUpdate.append(cube)
                cubeData[cube.id] = cube
        if cubesToUpdate:
            world.updateCubes(cubesToUpdate)
            world.replan(robot, grid)
        time.sleep(0.01) # yield

def cozmoBehavior(robot: cozmo.robot.Robot):
    """Cozmo search behavior. See assignment description for details

        Has global access to grid, a CozGrid instance created by the main thread, and
        stopevent, a threading.Event instance used to signal when the main thread has stopped.
        You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
        main thread to finish.

        Arguments:
        robot -- cozmo.robot.Robot instance, supplied by cozmo.run_program
    """

    global grid, stopevent

    robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE + cozmo.util.degrees(15.0)).wait_for_completed()

    world = World(robot.pose, Vector2.fromCell(grid.getStart()), grid.scale)
    world.replan(robot, grid)

    mapThread = threading.Thread(target=updateMapThread, name="Map Update", args=(robot, world, grid))
    movementThread = threading.Thread(target=updateMovementThread, name="Movement Update", args=(robot, world))

    mapThread.start()
    movementThread.start()

    mapThread.join()
    movementThread.join()


######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """
        
    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
    global grid, stopevent
    stopevent = threading.Event()
    grid = CozGrid("emptygrid.json")
    visualizer = Visualizer(grid)
    updater = UpdateThread(visualizer)
    updater.start()
    robot = RobotThread()
    robot.start()
    visualizer.start()
    stopevent.set()

