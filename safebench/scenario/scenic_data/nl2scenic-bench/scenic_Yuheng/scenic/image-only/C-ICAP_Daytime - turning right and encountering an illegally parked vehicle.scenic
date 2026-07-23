"""Scenario Description:

The scenario depicts a four-way intersection with two-way, two-lane roads where a green Vehicle Under Test (VUT) approaches from the bottom lane and intends to execute a right turn, as indicated by a blue curved arrow. As the VUT navigates the turn across a yellow hatched crosswalk area, its path is obstructed by two blue vehicles illegally parked in the lane it is turning into. The first parked vehicle is positioned 2 meters from the crosswalk markings, while the second vehicle is parked 1 meter behind the first, both situated 0.2 meters from the right curb, creating a blockage in the target lane.

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

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_INIT_DIST = [20, 30]
param EGO_SPEED = VerifaiRange(5, 8)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

PARKED_OFFSET_FROM_CURB = 0.2
PARKED1_DIST_FROM_CROSSWALK = 2.0
PARKED2_DIST_BEHIND_PARKED1 = 1.0

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 3
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego approaches from an incoming lane and makes a right turn
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Target lane is the end lane of the right turn maneuver
targetLane = egoManeuver.endLane

# Define spawn points for parked vehicles relative to the target lane
# Parked vehicles are offset from the right curb (right edge of the lane)
# We place them along the target lane centerline but shifted toward the right edge
targetLaneRightEdge = targetLane.rightEdge

# First parked car: 2m from crosswalk into the target lane
# We use a point on the target lane centerline near the start, then offset laterally
parked1BasePt = new OrientedPoint on targetLane.centerline,
    with position targetLane.centerline.pointAtDistance(PARKED1_DIST_FROM_CROSSWALK)
parked1Pt = new OrientedPoint at parked1BasePt offset by (0, -PARKED_OFFSET_FROM_CURB),
    with heading parked1BasePt.heading

# Second parked car: 1m behind the first (further back along the lane)
parked2BasePt = new OrientedPoint on targetLane.centerline,
    with position targetLane.centerline.pointAtDistance(PARKED1_DIST_FROM_CROSSWALK + PARKED2_DIST_BEHIND_PARKED1)
parked2Pt = new OrientedPoint at parked2BasePt offset by (0, -PARKED_OFFSET_FROM_CURB),
    with heading parked2BasePt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (0, 1, 0),
    with behavior EgoBehavior(egoTrajectory)

parkedCar1 = new Car at parked1Pt,
    with blueprint MODEL,
    with color (0, 0, 1),
    with velocity (0, 0, 0)

parkedCar2 = new Car at parked2Pt,
    with blueprint MODEL,
    with color (0, 0, 1),
    with velocity (0, 0, 0)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require parkedCar1 can see targetLane
require parkedCar2 can see targetLane

terminate when (distance to egoSpawnPt) > TERM_DIST