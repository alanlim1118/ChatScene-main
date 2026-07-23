"""Scenario Description:

A blue ego vehicle is depicted on a multi-lane road marked by dashed white lines, initially positioned in the rightmost lane. A curved blue arrow illustrates the vehicle's trajectory as it executes a continuous maneuver to change lanes to the left. The path shows the car crossing the dashed white lane markings, moving from the bottom lane upwards across the adjacent lanes, effectively shifting from the right side of the road towards the left lanes.

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
param OPT_EGO_SPEED = Range(5, 10)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed, lane_change_target):
    do FollowLaneBehavior(target_speed=target_speed) for 3 seconds
    do LaneChangeBehavior(laneSectionToSwitch=lane_change_target, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find rightmost forward driving lanes that have at least one lane to the left
rightmostLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward:
            hasLeft = laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward
            hasRight = laneSec._laneToRight is not None and laneSec._laneToRight.isForward
            if hasLeft and not hasRight:
                rightmostLaneSecs.append(laneSec)

egoLaneSec = Uniform(*rightmostLaneSecs)
targetLaneSec = egoLaneSec._laneToLeft
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    facing roadDirection,
    with blueprint EGO_MODEL,
    with color [0, 0, 255],
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        targetLaneSec
    )

require rightmostLaneSecs is not None
require distance to intersection >= 50