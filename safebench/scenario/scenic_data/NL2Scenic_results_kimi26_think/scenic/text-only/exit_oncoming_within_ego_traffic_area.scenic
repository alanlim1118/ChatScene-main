"""Scenario Description:

The ego car travels straight forward within its lane. Ahead of it, an oncoming adversarial object exits the ego's traffic area by veering into the adjacent lane.

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

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(5, 10)
param OPT_ADV_START_DIST = Range(20, 40)
param OPT_ADV_LANE_CHANGE_DIST = Range(10, 20)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdversarialBehavior():
    # Drive in the oncoming lane towards the ego
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until distance from self to ego < globalParameters.OPT_ADV_LANE_CHANGE_DIST
    # Veer into the adjacent lane to exit the ego's traffic area
    do LaneChangeBehavior(laneSectionToSwitch=advAdjacentLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select an intersection that supports straight maneuvers
intersection = Uniform(*filter(lambda i: any(m.type is ManeuverType.STRAIGHT for m in i.maneuvers), network.intersections))

# Ego straight maneuver
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: conflicting straight maneuver (oncoming)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m in egoManeuver.conflictingManeuvers, intersection.maneuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint following advInitLane.orientation from advInitLane.centerline.end for -globalParameters.OPT_ADV_START_DIST

# Find a lane section on the adversary's lane that has an adjacent lane
advLaneSec = None
for sec in advInitLane.sections:
    if sec._laneToLeft is not None or sec._laneToRight is not None:
        advLaneSec = sec
        break

require advLaneSec is not None

if advLaneSec._laneToLeft is not None:
    advAdjacentLaneSec = advLaneSec._laneToLeft
else:
    advAdjacentLaneSec = advLaneSec._laneToRight

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoInitLane,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn advInitLane,
    with behavior AdversarialBehavior()

require 40 <= (distance from egoSpawnPt to intersection) <= 60
require abs(egoSpawnPt.heading - advSpawnPt.heading) > 170 deg

terminate when distance from ego to AdvAgent > 70