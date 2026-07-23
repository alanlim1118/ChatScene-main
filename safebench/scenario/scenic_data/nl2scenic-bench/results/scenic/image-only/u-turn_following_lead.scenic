"""Scenario Description:

This top-down schematic of a four-way intersection depicts a traffic scenario on the southern approach involving two vehicles. A pink vehicle is positioned in the left lane, angled into the intersection with a purple curved arrow indicating a U-turn maneuver. Behind and to the right of the pink vehicle, a blue vehicle is situated in the adjacent right lane, with a blue curved arrow showing it initiating a left turn into the intersection, illustrating a situation where a following vehicle executes a U-turn behind a leading object.

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

MODEL_PINK = 'vehicle.lincoln.mkz_2017'
MODEL_BLUE = 'vehicle.tesla.model3'

EGO_INIT_DIST = [15, 25]
param EGO_SPEED = VerifaiRange(5, 8)
param EGO_BRAKE = VerifaiRange(0.5, 1.0)

ADV_INIT_DIST = [10, 20]
param ADV_SPEED = VerifaiRange(5, 8)

param SAFETY_DIST = VerifaiRange(8, 15)
CRASH_DIST = 4
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

# Pink vehicle (ego) in left lane performing U-turn
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.U_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Blue vehicle (adversary) in adjacent right lane performing left turn
# The right lane is adjacent to the ego's left lane at the same intersection approach
advInitLane = Uniform(*filter(lambda lane:
        lane is not egoInitLane and
        lane.road is egoInitLane.road and
        lane.successor is not None,
    intersection.incomingLanes))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL_PINK,
    with color (1.0, 0.4, 0.7),
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL_BLUE,
    with color (0.2, 0.3, 0.9),
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

# Ensure adversary is behind and to the right of ego
require distance from adversary to egoSpawnPt >= 5
require EGO_INIT_DIST[0] <= (distance from ego to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]

terminate when (distance from ego to egoSpawnPt) > TERM_DIST