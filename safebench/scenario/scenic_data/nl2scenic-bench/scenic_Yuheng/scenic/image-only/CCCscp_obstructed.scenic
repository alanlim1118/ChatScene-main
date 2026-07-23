"""Scenario Description:

This schematic depicts a traffic scenario at an intersection involving a potential side-impact collision between two vehicles. On the left side, there are two lanes of queued black vehicles; the upper lane contains three cars spaced 1 meter apart with the lead vehicle 21 meters from the intersection, while the lower lane has three similar cars spaced 1 meter apart, positioned 24 meters away. Between these queues, a white vehicle is positioned in a central lane, aligned with a horizontal red path indicating forward motion straight across the junction. Perpendicular to this, a vertical red path indicates a crossing trajectory for a vehicle traveling from the top of the image downwards, with another white vehicle visible on the vertical road segment at the bottom. The scenario describes a collision event where the vehicle under test, traveling along the straight horizontal path, moves towards the vehicle crossing on the perpendicular path, resulting in the frontal structure of the test vehicle striking the side of the crossing vehicle.

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

BLACK_CAR_MODEL = 'vehicle.tesla.model3'
WHITE_CAR_MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = 21
CROSSING_INIT_DIST = Uniform(15, 25)

QUEUE_SPACING = 1.0
UPPER_LANE_OFFSET = 21
LOWER_LANE_OFFSET = 24

param EGO_SPEED = VerifaiRange(8, 12)
param CROSSING_SPEED = VerifaiRange(8, 12)

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 3
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(1.0)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

behavior CrossingBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.CROSSING_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego vehicle: white car going straight horizontally
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Crossing vehicle: coming from perpendicular direction (top to bottom relative to ego)
crossingStartLane = Uniform(*filter(lambda m:
        m.type is ManeuverType.STRAIGHT,
        egoManeuver.conflictingManeuvers)
    ).startLane
crossingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, crossingStartLane.maneuvers))
crossingTrajectory = [crossingStartLane, crossingManeuver.connectingLane, crossingManeuver.endLane]
crossingSpawnPt = new OrientedPoint in crossingStartLane.centerline

# Identify adjacent lanes for queued vehicles
# Upper and lower lanes relative to ego's lane at the intersection
allIncomingLanes = list(intersection.incomingLanes)
adjacentLanes = [l for l in allIncomingLanes if l is not egoInitLane and l.road is egoInitLane.road]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle (white, center lane, going straight)
ego = new Car at egoSpawnPt,
    with blueprint WHITE_CAR_MODEL,
    with behavior EgoBehavior(egoTrajectory)

require abs((distance from ego to intersection) - EGO_INIT_DIST) < 2

# Crossing vehicle (perpendicular path, top to bottom)
crossing_car = new Car at crossingSpawnPt,
    with blueprint WHITE_CAR_MODEL,
    with behavior CrossingBehavior(crossingTrajectory)

require abs((distance from crossing_car to intersection) - CROSSING_INIT_DIST) < 5

# Queued black vehicles in upper adjacent lane (3 cars, 1m apart, lead 21m from intersection)
upperLane = Uniform(*adjacentLanes) if adjacentLanes else egoInitLane
upperLeadPt = new OrientedPoint in upperLane.centerline
upper_lead = new Car at upperLeadPt,
    with blueprint BLACK_CAR_MODEL,
    with behavior StopBehavior()
require abs((distance from upper_lead to intersection) - UPPER_LANE_OFFSET) < 2

upper_mid = new Car following roadDirection from upperLeadPt for -QUEUE_SPACING,
    with blueprint BLACK_CAR_MODEL,
    with behavior StopBehavior()

upper_rear = new Car following roadDirection from upper_mid for -QUEUE_SPACING,
    with blueprint BLACK_CAR_MODEL,
    with behavior StopBehavior()

# Queued black vehicles in lower adjacent lane (3 cars, 1m apart, lead 24m from intersection)
lowerLaneCandidates = [l for l in adjacentLanes if l is not upperLane]
lowerLane = Uniform(*lowerLaneCandidates) if lowerLaneCandidates else upperLane
lowerLeadPt = new OrientedPoint in lowerLane.centerline
lower_lead = new Car at lowerLeadPt,
    with blueprint BLACK_CAR_MODEL,
    with behavior StopBehavior()
require abs((distance from lower_lead to intersection) - LOWER_LANE_OFFSET) < 2

lower_mid = new Car following roadDirection from lowerLeadPt for -QUEUE_SPACING,
    with blueprint BLACK_CAR_MODEL,
    with behavior StopBehavior()

lower_rear = new Car following roadDirection from lower_mid for -QUEUE_SPACING,
    with blueprint BLACK_CAR_MODEL,
    with behavior StopBehavior()

# Termination condition
terminate when (distance from ego to egoSpawnPt) > TERM_DIST