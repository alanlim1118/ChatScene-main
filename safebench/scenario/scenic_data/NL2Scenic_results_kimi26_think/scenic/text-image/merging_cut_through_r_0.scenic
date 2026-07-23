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

param OPT_EGO_SPEED = Range(5, 7)
param OPT_ADV_SPEED = Range(6, 8)
param OPT_ADV_DIST = Range(25, 40)
param OPT_EGO_START_DIST = Range(3, 6)
param OPT_ADV_START_DIST = Range(3, 6)
param OPT_SAFETY_DIST = Range(5, 8)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, target_lane):
    try:
        do FollowLaneBehavior(target_speed=speed) until (distance from self to egoSpawnPt > globalParameters.OPT_EGO_START_DIST)
        do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=speed)
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_SAFETY_DIST):
        take SetBrakeAction(1)

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

laneSecsWithLeftAndRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToLeft is not None and
            laneSec._laneToLeft.isForward and
            laneSec._laneToRight is not None and
            laneSec._laneToRight.isForward
        ):
            laneSecsWithLeftAndRight.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftAndRight)
leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

leftLaneProj = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from leftLaneProj for globalParameters.OPT_ADV_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, rightLaneSec)

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, egoLaneSec, rightLaneSec)

require distance to intersection >= 100
terminate when distance from ego to egoSpawnPt > 150