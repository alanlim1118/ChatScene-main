"""Scenario Description:

The ego car executes a left turn at the intersection from the right lane. Simultaneously, an adversarial object entering parallel from the left lane proceeds straight ahead, illustrating the scenario of a parallel entry turning left with an object from the left passing straight.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(5, 10)
param OPT_START_DIST = Range(10, 25)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: left-turn maneuver starting from its lane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Adversary: straight maneuver through the same intersection
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

# Spawn both agents at the same distance from the intersection along their respective lanes
egoSpawnPt = new OrientedPoint following egoInitLane.orientation from egoInitLane.centerline.end for -globalParameters.OPT_START_DIST
advSpawnPt = new OrientedPoint following advInitLane.orientation from advInitLane.centerline.end for -globalParameters.OPT_START_DIST

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
    with behavior AdvBehavior()

# Enforce parallel entry from adjacent lanes:
# ego in the right lane and adversary in the left lane, side-by-side.
require egoInitLane != advInitLane
require abs(egoSpawnPt.heading - advSpawnPt.heading) < 10 deg
require 2 < (distance from egoSpawnPt to advSpawnPt) < 5
require 10 <= (distance from egoSpawnPt to intersection) <= 30