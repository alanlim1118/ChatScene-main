"""Scenario Description:

Under clear, sunny conditions in an urban environment, the ego vehicle proceeds along a multi-lane road separated by a tree-lined median, approaching a large roundabout with a central architectural structure. A red adversary vehicle travels in the adjacent left lane, moving concurrently alongside the ego vehicle as both approach the intersection. Upon reaching the entrance, both vehicles enter the roundabout simultaneously, with the red car occupying the inner circulating lane while the ego vehicle stays in the outer lane to execute a leftward path. The ego vehicle must maintain strict lane discipline and continuously monitor the adjacent red vehicle to safely navigate the curve and complete its turn through the roundabout without conflict.

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

EGO_INIT_DIST = [25, 30]
ADV_INIT_DIST = [25, 30]
param EGO_SPEED = VerifaiRange(7, 10)
param ADV_SPEED = VerifaiRange(7, 10)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

param SAFETY_DIST = VerifaiRange(5, 10)
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

behavior AdvBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select the largest intersection, assumed to be the central roundabout
intersection = max(network.intersections, key=lambda i: len(i.incomingLanes))

# Ego in a lane that has a left neighbor; both lanes support left-turn maneuvers through the roundabout
egoInitLane = Uniform(*filter(lambda l: l.leftLane is not None and 
    any(m.type is ManeuverType.LEFT_TURN for m in l.maneuvers) and
    any(m.type is ManeuverType.LEFT_TURN for m in l.leftLane.maneuvers), intersection.incomingLanes))
advInitLane = egoInitLane.leftLane

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with color Color(1, 0, 0),
    with behavior AdvBehavior(advTrajectory)

# Ensure both vehicles are alongside each other at a similar distance from the roundabout
require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require abs((distance to intersection) - (distance from adversary to intersection)) <= 2

terminate when (distance to egoSpawnPt) > TERM_DIST