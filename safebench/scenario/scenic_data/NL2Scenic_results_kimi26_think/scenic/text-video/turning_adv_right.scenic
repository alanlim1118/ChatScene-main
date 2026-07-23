"""Scenario Description:

Under overcast daylight conditions, the ego vehicle drives straight along a wide road bordered by large white buildings on the left and a construction site with colorful hoardings on the right. As the ego vehicle approaches an intersection, a black sedan turns right from the side road, cutting directly into the ego vehicle's path. The sedan unexpectedly decelerates immediately after merging, leaving the ego vehicle with insufficient time to react, resulting in a rear-end collision where the ego vehicle strikes the back of the black sedan and comes to a halt directly behind it.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town10HD'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town10HD'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(6, 9)
param OPT_BRAKE_DIST = Range(4, 7)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to AdvAgent < globalParameters.OPT_BRAKE_DIST):
        take SetBrakeAction(1)
        abort

behavior AdvBehavior():
    do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to mergePoint < 3)
    take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*network.intersections)

# Ego drives straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adversary turns right from a side road and merges into the ego's lane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.endLane == egoManeuver.endLane, intersection.maneuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn points
egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline
advSpawnPt = new OrientedPoint in advManeuver.startLane.centerline

require 40 <= (distance from egoSpawnPt to intersection) <= 60
require 10 <= (distance from advSpawnPt to intersection) <= 20

# Point just after the intersection in the adversary's end lane where it will brake
mergePoint = new OrientedPoint in advManeuver.endLane.centerline
require (distance from mergePoint to intersection) <= 5

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (0, 0, 0),
    with behavior AdvBehavior()

terminate when (ego.speed < 0.5 and AdvAgent.speed < 0.5 and distance from ego to AdvAgent < 5)