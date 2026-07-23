"""Scenario Description:

The ego vehicle (blue) is positioned in the center lane of a three-lane road and executes a lane change maneuver to the right. Simultaneously, a following vehicle (pink) in the left lane travels straight forward, positioned slightly behind the ego vehicle. This scenario represents a "Lane change right with following object" situation where the ego changes lanes while being followed by another vehicle in the adjacent left lane.

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
FOLLOWER_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_FOLLOWER_SPEED = Range(8, 12)
param OPT_FOLLOWER_DISTANCE = Range(10, 20)  # Follower is slightly behind ego

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLaneChangeRightBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed) for Range(2, 4) seconds
        do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, 5):
        take SetBrakeAction(1)

behavior FollowerBehavior(target_speed):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, 5):
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have both a left and right neighbor (i.e., center lane of 3+ lane road)
centerLaneSections = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is not None 
            and laneSec._laneToRight.isForward):
            centerLaneSections.append(laneSec)

require len(centerLaneSections) > 0

egoLaneSec = Uniform(*centerLaneSections)
rightLaneSec = egoLaneSec._laneToRight
leftLaneSec = egoLaneSec._laneToLeft

# Ego spawn point on center lane
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline

# Follower spawn point on left lane, slightly behind ego
leftLaneProjPt = leftLaneSec.centerline.project(egoSpawnPt.position)
followerSpawnPt = new OrientedPoint following roadDirection from leftLaneProjPt for -globalParameters.OPT_FOLLOWER_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (blue, center lane, changing to right) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (0, 0, 1, 1),  # Blue
    with behavior EgoLaneChangeRightBehavior(globalParameters.OPT_EGO_SPEED)

# --- Following vehicle (pink, left lane, traveling straight) ---
FollowerAgent = new Car at followerSpawnPt,
    with heading followerSpawnPt.heading,
    with regionContainedIn leftLaneSec,
    with blueprint FOLLOWER_MODEL,
    with color (1, 0.4, 0.7, 1),  # Pink
    with behavior FollowerBehavior(globalParameters.OPT_FOLLOWER_SPEED)

require distance to intersection >= 80  # Ensure sufficient road length for lane change