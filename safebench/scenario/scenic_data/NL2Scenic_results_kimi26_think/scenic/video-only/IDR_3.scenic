"""Scenario Description:

The ego vehicle travels straight through a four-way urban intersection, passing between tall buildings on either side. As the ego vehicle moves through the junction, a red adversary vehicle approaches from the left cross-street and executes a left turn onto the same road ahead of the ego vehicle. The red vehicle completes the turn and proceeds northbound in the lane directly in front of the ego vehicle's path. Another white vehicle is visible waiting at the stop line on the left cross-street behind the turning red car. The ego vehicle continues forward, maintaining its lane as it follows the red vehicle that has just merged onto the roadway.

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

ADV_INIT_DIST = [8, 12]
param ADV_SPEED = VerifaiRange(6, 9)

WHITE_INIT_DIST = [20, 28]

param SAFETY_DIST = VerifaiRange(10, 20)
CRASH_DIST = 5
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

# Ego travels straight through the intersection
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: left turn from the left cross-street onto the same outgoing lane ahead of ego
advManeuver = Uniform(*[m for lane in intersection.incomingLanes for m in lane.maneuvers
                         if lane != egoInitLane
                         and m.type is ManeuverType.LEFT_TURN
                         and m.endLane == egoManeuver.endLane])
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# White vehicle waiting on the same left cross-street behind the adversary
whiteSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with color (255, 0, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

whiteCar = new Car at whiteSpawnPt,
    with blueprint MODEL,
    with color (255, 255, 255)

# Placement constraints
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require WHITE_INIT_DIST[0] <= (distance from whiteCar to intersection) <= WHITE_INIT_DIST[1]
require (distance from adversary to intersection) < (distance from whiteCar to intersection)

terminate when (distance to egoSpawnPt) > TERM_DIST