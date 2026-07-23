"""Scenario Description:

In a top-down schematic view of a three-lane road, a blue ego vehicle travels straight forward in the center lane, indicated by a straight blue arrow pointing ahead. To its right and slightly behind, a pink adversarial vehicle executes a cut-through maneuver, crossing directly in front of the ego vehicle's path from the right lane towards the left lane, as illustrated by a curved pink trajectory arrow that intersects the center lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(5, 8)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED + Range(2, 4)
param OPT_ADV_BEHIND_DIST = Range(3, 8)

#################################
# AGENT BEHAVIORS               #
#################################

behavior CutThroughBehavior(target_speed, center_lane, left_lane):
    do FollowLaneBehavior(target_speed=target_speed)
    do LaneChangeBehavior(laneSectionToSwitch=center_lane, target_speed=target_speed)
    do LaneChangeBehavior(laneSectionToSwitch=left_lane, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
rightLaneSec = egoLaneSec._laneToRight
leftLaneSec = egoLaneSec._laneToLeft

# Adversarial in right lane, slightly behind ego
rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from rightLanePt for -globalParameters.OPT_ADV_BEHIND_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL

AdvAgent = new Car at AdvSpawnPt,
    with heading AdvSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior CutThroughBehavior(globalParameters.OPT_ADV_SPEED, egoLaneSec, leftLaneSec)

require distance to intersection >= 100
