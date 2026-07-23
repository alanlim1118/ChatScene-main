"""Scenario Description:

Ego vehicle follows a red lead vehicle in the right-hand lane approaching a 4-way intersection. The red vehicle is stationary at the stop line, temporarily blocking the lane ahead. The ego vehicle holds its position and waits for the red vehicle to accelerate and clear the stop line, then executes a left lane change at the junction by exiting the intersection into the left lane and continues.

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

EGO_INIT_DIST = [10, 20]
param EGO_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

LEAD_INIT_DIST = [0, 3]
param LEAD_SPEED = VerifaiRange(5, 8)
param LEAD_WAIT_TIME = VerifaiRange(3, 6)

param SAFETY_DIST = VerifaiRange(8, 12)
CRASH_DIST = 3
TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryBehavior():
    while True:
        take SetBrakeAction(1.0)

behavior LeadBehavior(trajectory):
    do StationaryBehavior() for globalParameters.LEAD_WAIT_TIME
    do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=trajectory)

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

# Ego and lead start in the same right-hand incoming lane
egoInitLane = Uniform(*intersection.incomingLanes)

# Ensure there is a left adjacent lane for the lane change
leftLane = egoInitLane.leftLane
require leftLane is not None

# Straight maneuver for the lead vehicle (remains in right lane)
leadManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
leadTrajectory = [egoInitLane, leadManeuver.connectingLane, leadManeuver.endLane]

# Straight maneuver for the left lane (ego's lane change target)
leftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, leftLane.maneuvers))
# Ego trajectory: right incoming -> intersection -> left outgoing (lane change at junction)
egoTrajectory = [egoInitLane, leadManeuver.connectingLane, leftManeuver.endLane]

# Spawn points along the lane centerline
leadSpawnPt = new OrientedPoint in egoInitLane.centerline
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

lead_vehicle = new Car at leadSpawnPt,
    with blueprint MODEL,
    with color [1, 0, 0],
    with behavior LeadBehavior(leadTrajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color [0, 0, 1],
    with behavior EgoBehavior(egoTrajectory)

# Positioning requirements
require LEAD_INIT_DIST[0] <= (distance from lead_vehicle to intersection) <= LEAD_INIT_DIST[1]
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require (distance to intersection) >= (distance from lead_vehicle to intersection) + 5

terminate when (distance to egoSpawnPt) > TERM_DIST