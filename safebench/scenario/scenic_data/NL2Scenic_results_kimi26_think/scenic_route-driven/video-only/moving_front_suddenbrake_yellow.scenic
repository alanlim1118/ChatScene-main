"""Scenario Description:

Under overcast daylight conditions, the ego vehicle travels forward in the center lane of a wide urban road, following a black sedan towards a large intersection flanked by blue construction barriers. As a large bus overtakes in the right lane, the lead vehicle continues forward, crossing the stop line and entering the intersection area. Suddenly, the lead vehicle's brake lights illuminate as it anchors its brakes abruptly, likely in response to a changing traffic signal, despite already being committed to the intersection. This unexpected halt forces the ego vehicle into a sudden emergency braking scenario, resulting in a rear-end collision with the sedan directly ahead.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
SEDAN_MODEL = "vehicle.audi.a2"
BUS_MODEL = "vehicle.carlamotors.carlacola"

param OPT_LEAD_SPEED = Range(10, 14)
param OPT_BUS_SPEED = Range(16, 20)
param OPT_START_DIST = Range(10, 15)
param OPT_BUS_OFFSET = Range(-30, -20)

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadBehavior(target_intersection):
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_SPEED) until self in target_intersection
    take SetThrottleAction(0), SetBrakeAction(1)
    wait

behavior BusBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_BUS_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way, egoLane.maneuvers))
intersection = egoManeuver.intersection

# Lead sedan ahead of ego by the same distance the original placed ego behind it
sedanSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_START_DIST

busLongPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_BUS_OFFSET
busSpawnPt = new OrientedPoint right of busLongPt by 4

#################################
# SCENARIO SPECIFICATION        #
#################################

leadVehicle = new Car at sedanSpawnPt,
    with heading sedanSpawnPt.heading,
    with regionContainedIn None,
    with blueprint SEDAN_MODEL,
    with color Color(0, 0, 0),
    with behavior LeadBehavior(intersection)

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL

bus = new Car at busSpawnPt,
    with heading busSpawnPt.heading,
    with regionContainedIn None,
    with blueprint BUS_MODEL,
    with behavior BusBehavior()

require 20 <= (distance from sedanSpawnPt to intersection) <= 40
require 10 <= (distance from busSpawnPt to intersection) <= 50

terminate after 30 seconds
