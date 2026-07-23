"""Scenario Description:

In a top-down schematic of a three-lane roadway, a blue ego vehicle positioned in the center lane begins a lane change maneuver towards the right lane, indicated by a curving blue arrow. Simultaneously, a pink adversarial vehicle located in the left lane and positioned further ahead executes a merging cut-through maneuver, crossing directly from the left lane into the right lane as indicated by a pink arrow, thereby targeting the same lane as the ego vehicle.

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

param OPT_ADV_SPEED = Range(6, 8)
param OPT_ADV_DIST = Range(25, 40)
param OPT_ADV_START_DIST = Range(3, 6)
param OPT_SAFETY_DIST = Range(5, 8)

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdvBehavior(speed, center_lane, right_lane):
    try:
        do FollowLaneBehavior(target_speed=speed) until (distance from self to advSpawnPt > globalParameters.OPT_ADV_START_DIST)
        do LaneChangeBehavior(laneSectionToSwitch=center_lane, target_speed=speed)
        do LaneChangeBehavior(laneSectionToSwitch=right_lane, target_speed=speed)
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_SAFETY_DIST):
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

leftLaneProj = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from leftLaneProj for globalParameters.OPT_ADV_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, egoLaneSec, rightLaneSec)

require distance to intersection >= 100
terminate when distance from ego to egoSpawnPt > 150
