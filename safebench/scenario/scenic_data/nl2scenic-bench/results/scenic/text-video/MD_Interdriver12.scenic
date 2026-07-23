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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
RED_CAR_MODEL = 'vehicle.tesla.model3'
WHITE_CAR_MODEL = 'vehicle.audi.a2'

EGO_INIT_DIST = [15, 25]
param EGO_SPEED = VerifaiRange(5, 8)
param EGO_BRAKE = VerifaiRange(0.6, 1.0)

RED_INIT_DIST = [30, 45]
WHITE_INIT_DIST_OFFSET = [12, 18]
param ADV_SPEED = VerifaiRange(8, 12)

param SAFETY_DIST = VerifaiRange(12, 20)
CRASH_DIST = 5
TERM_DIST = 80

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoYieldBehavior(trajectory, yieldVehicles):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when any(withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST) for v in yieldVehicles):
        take SetBrakeAction(globalParameters.EGO_BRAKE)
    interrupt when withinDistanceToAnyObjs(self, CRASH_DIST):
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a T-intersection where ego comes from the stem and merges onto the cross road
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego approaches from the minor road (stem of T)
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type in (ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN), egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversaries travel on the main road in the lane ego intends to enter
# They come from the right relative to ego's merge direction
advInitLane = egoManeuver.endLane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

redSpawnPt = new OrientedPoint in advInitLane.centerline
whiteSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior EgoYieldBehavior(egoTrajectory, [redCar, whiteCar])

redCar = new Car at redSpawnPt,
    with blueprint RED_CAR_MODEL,
    with color (1, 0, 0),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

whiteCar = new Car at whiteSpawnPt,
    with blueprint WHITE_CAR_MODEL,
    with color (1, 1, 1),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Spatial constraints
require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require RED_INIT_DIST[0] <= (distance from redCar to intersection) <= RED_INIT_DIST[1]
require WHITE_INIT_DIST_OFFSET[0] <= (distance from whiteCar to redCar) <= WHITE_INIT_DIST_OFFSET[1]

# Ensure adversaries are positioned so they approach from right-to-left relative to ego
require (relative position of redCar from ego).y > 0 or (relative position of redCar from ego).x < 0

terminate when (distance from ego to egoSpawnPt) > TERM_DIST