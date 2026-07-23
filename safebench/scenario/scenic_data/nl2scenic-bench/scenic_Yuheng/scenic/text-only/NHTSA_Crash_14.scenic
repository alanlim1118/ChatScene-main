"""Scenario Description:

Vehicle A and vehicle B were both heading in the same direction on a multi-lane road in different lanes. B attempted to turn from the curb lane across the path of A onto a side street. Driver A struck illegally turning B in the driver's side.

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

param EGO_SPEED = Range(8, 12)       # Vehicle A (straight, faster)
param ADV_SPEED = Range(4, 7)        # Vehicle B (turning, slower)
param EGO_INIT_DIST = Range(30, 45)  # Distance of A from intersection
param ADV_INIT_DIST = Range(15, 25)  # Distance of B from intersection (ahead of A)
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, 5):
        take SetBrakeAction(1.0)
    terminate

behavior AdvTurnBehavior(trajectory):
    """Vehicle B turns left across ego's path onto side street."""
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a 3-way or 4-way intersection where a left turn crosses a straight path
intersection = Uniform(*filter(lambda i: i.is3Way or i.is4Way, network.intersections))

# Vehicle B (adversary) performs a LEFT_TURN from the curb (rightmost) lane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Vehicle A (ego) goes STRAIGHT through the same intersection, in a lane to the left of B
# We find straight maneuvers that conflict with B's left turn (i.e., same direction, adjacent lane)
egoManeuver = Uniform(*filter(
    lambda m: m.type is ManeuverType.STRAIGHT and m.startLane is not advInitLane,
    advManeuver.conflictingManeuvers
))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Ensure both vehicles are heading in the same direction
require abs(egoSpawnPt.heading - advSpawnPt.heading) < 15 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvTurnBehavior(advTrajectory)

# Vehicle A is behind Vehicle B initially (B is closer to intersection)
require EGO_INIT_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from advSpawnPt to intersection) <= ADV_INIT_DIST[1]

# Ensure B starts ahead of A (closer to intersection)
require (distance from advSpawnPt to intersection) < (distance from egoSpawnPt to intersection)

terminate when (distance from ego to egoSpawnPt) > TERM_DIST