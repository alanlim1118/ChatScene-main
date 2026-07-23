"""Scenario Description:

The video presents a top-down simulation view of a traffic scenario where the ego vehicle, represented by a red rectangle, accelerates from a standstill to over 100 km/h while executing a lane change. Initially positioned in the rightmost lane adjacent to a blue area representing the road boundary, the ego vehicle moves laterally to the left, crossing dashed white lane markings to merge into the adjacent lane. A white vehicle with black stripes is visible traveling in the lane further to the left, maintaining its lane as the ego vehicle maneuvers. The accompanying text describes this action as an optional lane change performed to optimize driving parameters such as speed and comfort, rather than being a mandatory navigational requirement.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
NPC_MODEL = "vehicle.tesla.model3"

# 100 km/h ≈ 27.78 m/s
param OPT_EGO_TARGET_SPEED = Range(26, 29)
param OPT_NPC_SPEED = Range(18, 22)
param OPT_LANE_CHANGE_TRIGGER_DIST = Range(30, 50)
param OPT_NPC_DISTANCE = Range(40, 70)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoOptionalLaneChangeBehavior(target_speed, trigger_dist, lane_change_target):
    """
    Ego accelerates from standstill and performs an optional lane change
    to the left for speed/comfort optimization (not mandatory).
    """
    try:
        do FollowLaneBehavior(target_speed=target_speed) until (distance from self to egoSpawnPt > trigger_dist)
        do LaneChangeBehavior(laneSectionToSwitch=lane_change_target, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, 5):
        take SetBrakeAction(1)

behavior NPCMaintainLaneBehavior(speed):
    """NPC maintains its lane at a constant speed."""
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find rightmost forward lane sections that have a left neighbor
rightmostLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is None):
            rightmostLaneSecs.append(laneSec)

require len(rightmostLaneSecs) > 0

egoLaneSec = Uniform(*rightmostLaneSecs)
targetLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place NPC in the target (left) lane ahead of ego
npcLanePt = targetLaneSec.centerline.project(egoSpawnPt.position)
NpcSpawnPt = new OrientedPoint following roadDirection from npcLanePt for globalParameters.OPT_NPC_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (rightmost lane, accelerates and optionally changes left) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (1.0, 0.0, 0.0),
    with behavior EgoOptionalLaneChangeBehavior(
        globalParameters.OPT_EGO_TARGET_SPEED,
        globalParameters.OPT_LANE_CHANGE_TRIGGER_DIST,
        targetLaneSec
    )

# --- NPC vehicle (left lane, maintains lane) ---
NpcAgent = new Car at NpcSpawnPt,
    with heading NpcSpawnPt.heading,
    with regionContainedIn targetLaneSec,
    with blueprint NPC_MODEL,
    with color (1.0, 1.0, 1.0),
    with behavior NPCMaintainLaneBehavior(globalParameters.OPT_NPC_SPEED)

require distance to intersection >= 100
terminate when distance from ego to egoSpawnPt > 200