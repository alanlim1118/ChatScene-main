"""Scenario Description:

The ego car drives straight forward through the intersection. In doing so, it passes an adversarial object moving parallel to it on the adjacent right lane, fulfilling the "pass object on the right parallel in intersection" scenario.

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

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(3, 6)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advInitLane = egoInitLane.rightLane
require advInitLane is not None

# Ensure the adjacent right lane also continues straight through the intersection
advManeuvers = [m for m in intersection.maneuvers if (m.type is ManeuverType.STRAIGHT) and (m.startLane == advInitLane)]
require len(advManeuvers) > 0

advSpawnPt = new OrientedPoint in advInitLane.centerline

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

require 25 <= (distance from egoSpawnPt to intersection) <= 35
require 5 <= (distance from advSpawnPt to intersection) <= 15
require abs(egoDir - advDir) < 10 deg