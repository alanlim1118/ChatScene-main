"""Scenario Description:

In a clear, sunny suburban environment viewed from a high angle, a blue ego vehicle travels along a narrow, one-way connecting road approaching a T-intersection with a larger main road. The ego vehicle intends to merge onto the main road but yields the right-of-way as it approaches the junction. Two other vehicles, a red car followed by a white car, are traveling along the main road from right to left in the lane the ego vehicle intends to enter. The ego vehicle slows and waits at the intersection line for both the red and white vehicles to pass, ensuring the path is clear before completing its merge maneuver.

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

EGO_INIT_DIST = [15, 20]
param EGO_SPEED = VerifaiRange(5, 8)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

RED_INIT_DIST = [10, 15]
WHITE_INIT_DIST = [25, 35]
param ADV_SPEED = VerifaiRange(8, 12)

param SAFETY_DIST = VerifaiRange(10, 15)
CRASH_DIST = 5
TERM_DIST = 70

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

intersection = Uniform(*filter(lambda i: len(i.incomingLanes) == 3, network.intersections))

# Ego is on the connecting road (stem of the T) and turns onto the main road
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type in (ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN), egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversaries on the main road traveling straight through the intersection
# into the same lane the ego intends to enter
advInitLane = Uniform(*filter(lambda l: any(m.type is ManeuverType.STRAIGHT and m.endLane is egoManeuver.endLane for m in l.maneuvers), intersection.incomingLanes))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.endLane is egoManeuver.endLane, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
redSpawnPt = new OrientedPoint in advInitLane.centerline
whiteSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with color (0, 0, 1),
    with behavior EgoBehavior(egoTrajectory)

redCar = new Car at redSpawnPt,
    with blueprint MODEL,
    with color (1, 0, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

whiteCar = new Car at whiteSpawnPt,
    with blueprint MODEL,
    with color (1, 1, 1),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require RED_INIT_DIST[0] <= (distance from redCar to intersection) <= RED_INIT_DIST[1]
require WHITE_INIT_DIST[0] <= (distance from whiteCar to intersection) <= WHITE_INIT_DIST[1]
require (distance from whiteCar to intersection) > (distance from redCar to intersection)
require (distance from redCar to whiteCar) > 8

terminate when (distance to egoSpawnPt) > TERM_DIST