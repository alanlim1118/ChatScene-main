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

param OPT_OTHER_SPEED = Range(20, 28)  # m/s, traffic in the further left lane

#################################
# AGENT BEHAVIORS               #
#################################

behavior OtherBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
targetLaneSec = egoLaneSec._laneToLeft
furtherLeftLaneSec = targetLaneSec._laneToLeft

# Place the other vehicle in the lane further to the left, slightly ahead or abreast
furtherLeftProj = furtherLeftLaneSec.centerline.project(egoSpawnPt.position)
otherSpawnPt = new OrientedPoint following roadDirection from furtherLeftProj for Range(-5, 20)

#################################
# SCENARIO SPECIFICATION        #
#################################

# --- Ego vehicle (rightmost lane, accelerates and changes left) ---
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL

# --- Other vehicle (lane further to the left, maintains lane) ---
other = new Car at otherSpawnPt,
    with regionContainedIn furtherLeftLaneSec,
    with behavior OtherBehavior(globalParameters.OPT_OTHER_SPEED)

require distance to intersection >= 100  # Ensure the ego vehicle is far from the intersection
terminate when distance from ego to egoSpawnPt > 200  # Terminate after sufficient distance
