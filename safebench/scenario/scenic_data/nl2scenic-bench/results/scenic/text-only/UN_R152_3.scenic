"""Scenario Description:

The subject vehicle drives a small radius curved road of which the guard pipes are constructed to the outer side, and a stationary vehicle (M1 category), a stationary pedestrian target or a stationary bicycle target is positioned just outside of the guard pipes and where on the extension of the centre of the lane.

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

param OPT_EGO_SPEED = Range(3, 8)
param OPT_BRAKE_DIST = Range(6, 10)
param OPT_GUARD_PIPE_OFFSET = Range(2.5, 4.0)  # Distance from lane edge to guard pipe
param OPT_TARGET_OFFSET = Range(0.5, 1.5)     # Additional offset beyond guard pipe for target

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a curved road segment with sufficient curvature (small radius)
allLanes = network.lanes
curvedLanes = filter(lambda l: l.centerline.length > 20 and abs(l.centerline.curvatureAt(0.5)) > 0.01, allLanes)
egoLane = Uniform(*curvedLanes)

# Pick a point along the centerline of the curved lane for ego spawn
egoSpawnPt = new OrientedPoint in egoLane.centerline

# Define a reference point further along the curve where the obstacle will be placed
# Use a parametric position along the centerline to place the target ahead of ego
targetParam = Range(0.4, 0.8)
targetCenterPt = new OrientedPoint at egoLane.centerline.pointAt(targetParam),
    with heading egoLane.centerline.headingAt(targetParam)

# Compute the outward direction from the curve (outer side of the curve)
# For a left-curving road, outer side is right; for right-curving, outer side is left
# We use the lane's rightEdge as proxy for outer side when curvature is positive
curveSign = egoLane.centerline.curvatureAt(targetParam)
outerDirection = targetCenterPt.heading + (90 deg if curveSign >= 0 else -90 deg)

# Place the stationary target just outside the guard pipes on the outer side
totalOffset = globalParameters.OPT_GUARD_PIPE_OFFSET + globalParameters.OPT_TARGET_OFFSET
obstacleSpawnPt = new OrientedPoint at targetCenterPt offset along outerDirection by totalOffset,
    with heading targetCenterPt.heading

# Randomly choose obstacle type: M1 vehicle, pedestrian, or bicycle
obstacleType = Uniform('car', 'pedestrian', 'bicycle')

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Stationary M1 category vehicle
obstacleCar = new Car at obstacleSpawnPt,
    with heading obstacleSpawnPt.heading,
    with regionContainedIn None,
    with behavior WaitBehavior(),
    when obstacleType == 'car'

# Stationary pedestrian target
obstaclePed = new Pedestrian at obstacleSpawnPt,
    with heading obstacleSpawnPt.heading,
    with regionContainedIn None,
    with behavior WaitBehavior(),
    when obstacleType == 'pedestrian'

# Stationary bicycle target
obstacleBike = new Bicycle at obstacleSpawnPt,
    with heading obstacleSpawnPt.heading,
    with regionContainedIn None,
    with behavior WaitBehavior(),
    when obstacleType == 'bicycle'

# Ensure ego spawns sufficiently before the obstacle
require distance from egoSpawnPt to obstacleSpawnPt >= 30
require distance from egoSpawnPt to obstacleSpawnPt <= 80