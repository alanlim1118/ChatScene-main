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

EGO_INIT_DIST = [20, 25]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, 5):
        take SetBrakeAction(globalParameters.EGO_BRAKE)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Target lane after completing the right turn
targetLane = egoManeuver.endLane

#################################
# PARKED VEHICLE POSITIONS      #
#################################

# First parked vehicle: base point on the right edge of the target lane,
# then offset 0.2 m laterally into the lane (away from the curb).
parked1Base = new OrientedPoint on targetLane.rightEdge
parked1Pt = new OrientedPoint at parked1Base, offset by 0 @ 0.2
# Restrict to approximately 2 meters from the intersection crosswalk markings
require 1.8 <= (distance from parked1Pt to intersection) <= 2.2

# Second parked vehicle: 1 meter behind (further along) the first vehicle
parked2Pt = new OrientedPoint at parked1Pt, offset by 1 @ 0

#################################
# SCENARIO SPECIFICATION        #
#################################

# VUT (Ego) - green vehicle executing the right turn
ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color Color(0, 1, 0),
    with behavior EgoBehavior(egoTrajectory)

# First illegally parked vehicle (blue)
parked1 = new Car at parked1Pt,
    with blueprint MODEL,
    with color Color(0, 0, 1)

# Second illegally parked vehicle (blue)
parked2 = new Car at parked2Pt,
    with blueprint MODEL,
    with color Color(0, 0, 1)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST