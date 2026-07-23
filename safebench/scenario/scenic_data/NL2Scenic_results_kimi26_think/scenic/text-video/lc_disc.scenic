"""Scenario Description:

The ego vehicle accelerates from a standstill to over 100 km/h while executing an optional lane change from the rightmost lane to the adjacent left lane to optimize driving speed and comfort. A second vehicle travels in the lane further to the left, maintaining its lane as the ego vehicle maneuvers.

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

param OPT_EGO_SPEED = Range(28, 35)    # m/s (~100-130 km/h)
param OPT_OTHER_SPEED = Range(20, 28)  # m/s, traffic in the further left lane

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed, lane_change_target):
    # Accelerate from standstill while changing to the target lane
    do LaneChangeBehavior(laneSectionToSwitch=lane_change_target, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

behavior OtherBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find rightmost forward lanes that have at least two lanes to their left
rightmostLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        left = laneSec._laneToLeft
        if (laneSec.isForward and 
            left is not None and left.isForward and
            left._laneToLeft is not None and left._laneToLeft.isForward and
            (laneSec._laneToRight is None or not laneSec._laneToRight.isForward)):
            rightmostLaneSecs.append(laneSec)

egoLaneSec = Uniform(*rightmostLaneSecs)
targetLaneSec = egoLaneSec._laneToLeft
furtherLeftLaneSec = targetLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place the other vehicle in the lane further to the left, slightly ahead or abreast
furtherLeftProj = furtherLeftLaneSec.centerline.project(egoSpawnPt.position)
otherSpawnPt = new OrientedPoint following roadDirection from furtherLeftProj for Range(-5, 20)

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (rightmost lane, accelerates and changes left) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        targetLaneSec
    )

# --- Other vehicle (lane further to the left, maintains lane) ---
other = new Car at otherSpawnPt,
    with regionContainedIn furtherLeftLaneSec,
    with behavior OtherBehavior(globalParameters.OPT_OTHER_SPEED)

require distance to intersection >= 100  # Ensure the ego vehicle is far from the intersection
terminate when distance from ego to egoSpawnPt > 200  # Terminate after sufficient distance