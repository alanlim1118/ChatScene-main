"""Scenario Description:

The scenario takes place on a long straight road with three lanes separated by white dashed lines, where a blue vehicle labeled VUT is driving in the middle lane behind a red vehicle labeled VT. The red vehicle is shown cutting out urgently to the left lane, indicated by a curved red arrow, while further ahead in the right lane, a half-squatting child and an orange dog are positioned as obstacles.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
VT_MODEL = "vehicle.tesla.model3"
LANE_WIDTH = 3.7

param OPT_VUT_SPEED = Range(5, 10)
param OPT_VT_SPEED = Range(5, 10)
param OPT_VT_DISTANCE = Range(15, 25)
param OPT_OBSTACLE_DISTANCE = Range(50, 80)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior FollowAndBrakeBehavior(speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=brake_dist):
        take SetBrakeAction(1)
        do WaitBehavior()

behavior CutOutLeftBehavior(speed):
    # Drive in the middle lane briefly before cutting out
    do FollowLaneBehavior(target_speed=speed) for 2 seconds
    # Urgent cut-out to the left lane
    take SetSteerAction(-0.4)
    do WaitBehavior() for 1.2 seconds
    take SetSteerAction(0.0)
    # Continue driving in the left lane
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and not i.isSignalized, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))

# Reference point in the middle lane
egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline
roadDir = egoInitLane.orientation

# VT spawn point ahead in the middle lane
vtSpawnPt = new OrientedPoint following roadDir from egoSpawnPt for globalParameters.OPT_VT_DISTANCE

# Obstacles further ahead in the right lane
obstacleBasePt = new OrientedPoint following roadDir from vtSpawnPt for globalParameters.OPT_OBSTACLE_DISTANCE
childSpawnPt = new OrientedPoint right of obstacleBasePt by LANE_WIDTH
dogSpawnPt = new OrientedPoint following roadDir from childSpawnPt for 2.0

#################################
# SCENARIO SPECIFICATION        #
#################################

# VUT (blue) in the middle lane
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color Color.withBytes([0, 0, 255]),
    with behavior FollowAndBrakeBehavior(globalParameters.OPT_VUT_SPEED, 10),
    with regionContainedIn None

# VT (red) ahead in the middle lane, then cuts urgently to the left lane
VT = new Car at vtSpawnPt,
    with blueprint VT_MODEL,
    with color Color.withBytes([255, 0, 0]),
    with behavior CutOutLeftBehavior(globalParameters.OPT_VT_SPEED),
    with regionContainedIn None

# Child obstacle in the right lane (static)
Child = new Pedestrian at childSpawnPt,
    with heading roadDir,
    with behavior WaitBehavior(),
    with regionContainedIn None

# Dog obstacle in the right lane (orange, static)
Dog = new Pedestrian at dogSpawnPt,
    with heading roadDir,
    with color Color.withBytes([255, 165, 0]),
    with behavior WaitBehavior(),
    with regionContainedIn None

require 40 <= (distance to intersection) <= 60
terminate when distance from ego to intersection > 80