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
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(6, 8)
param OPT_ADV_SPEED = Range(7, 9)
param OPT_EGO_SAFETY_DISTANCE = Range(6, 9)
param OPT_ADV_AHEAD_DIST = Range(25, 40)       # Adversary starts this far ahead of ego (longitudinally)
param OPT_EGO_LANE_CHANGE_TRIGGER = Range(15, 25)  # Ego initiates lane change after traveling this distance
param OPT_ADV_CUT_TRIGGER_DIST = Range(18, 30)     # Adversary initiates cut when within this distance to ego

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(ego_speed, safety_distance, trigger_dist):
    try:
        do FollowLaneBehavior(target_speed=ego_speed) until (distance from self to egoSpawnPt > trigger_dist)
        do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, target_speed=ego_speed)
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when withinDistanceToObjsInLane(self, safety_distance):
        take SetBrakeAction(1)

behavior AdvCutThroughBehavior(adv_speed, cut_trigger_dist):
    try:
        do FollowLaneBehavior(target_speed=adv_speed) until (distance from self to ego < cut_trigger_dist)
        # Cut through: two consecutive lane changes from left lane to right lane (crossing center lane)
        do LaneChangeBehavior(laneSectionToSwitch=centerLaneSec, target_speed=adv_speed)
        do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, target_speed=adv_speed)
        do FollowLaneBehavior(target_speed=adv_speed)
    interrupt when withinDistanceToObjsInLane(self, 5):
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have both a left neighbor and a right neighbor (three-lane road)
laneSecsWithBothNeighbors = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward
            and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is not None and laneSec._laneToRight.isForward):
            laneSecsWithBothNeighbors.append(laneSec)

require len(laneSecsWithBothNeighbors) > 0

centerLaneSec = Uniform(*laneSecsWithBothNeighbors)
targetLaneSec = centerLaneSec._laneToRight   # Right lane (ego's target)
leftLaneSec = centerLaneSec._laneToLeft      # Left lane (adversary's start)

# Ego spawns in center lane
egoSpawnPt = new OrientedPoint in centerLaneSec.centerline

# Adversary spawns in left lane, ahead of ego
leftLaneProj = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from leftLaneProj for globalParameters.OPT_ADV_AHEAD_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (center lane, changing to right) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn centerLaneSec,
    with blueprint EGO_MODEL,
    with color (0, 0, 1, 1),  # Blue
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_SAFETY_DISTANCE,
        globalParameters.OPT_EGO_LANE_CHANGE_TRIGGER
    )

# --- Adversarial vehicle (left lane, cutting through to right) ---
AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7, 1),  # Pink
    with behavior AdvCutThroughBehavior(
        globalParameters.OPT_ADV_SPEED,
        globalParameters.OPT_ADV_CUT_TRIGGER_DIST
    )

require distance to intersection >= 100
terminate when distance from ego to egoSpawnPt > 150