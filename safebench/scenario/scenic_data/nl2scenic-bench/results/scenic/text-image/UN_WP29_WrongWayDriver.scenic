"""Scenario Description:

This traffic scenario illustrates an oncoming situation on a straight road divided into two lanes by a dashed center line. At the start of the sequence, labeled "@oncoming start," a green ego vehicle is positioned in the top lane traveling to the left, while a red vehicle is in the bottom lane traveling to the right, approaching the ego vehicle from the opposite direction. The sequence concludes at the "@oncoming end" stage, where the red vehicle has successfully passed the green ego vehicle, and both cars continue driving straight in their respective lanes away from each other.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

param EGO_SPEED = Range(8, 12)
param ADV_SPEED = Range(8, 12)
param INITIAL_SEPARATION = Range(60, 90)
param PASS_COMPLETE_DIST = 15

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoOncomingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

behavior AdvOncomingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find straight road sections with exactly two opposing lanes (one forward, one backward)
candidateLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            leftLane = laneSec._laneToLeft
            if leftLane.isForward != laneSec.isForward:
                candidateLaneSecs.append(laneSec)

require len(candidateLaneSecs) > 0

egoLaneSec = Uniform(*candidateLaneSecs)
advLaneSec = egoLaneSec._laneToLeft

# Place ego in its lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place adversary in the opposite lane, facing toward ego, at initial separation distance
advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.INITIAL_SEPARATION,
    offsetRotated by 180 deg

# Ensure adversary is actually in the correct opposing lane region
require advSpawnPt in advLaneSec

#################################
# SCENARIO SPECIFICATION        #
#################################

# @oncoming start: Ego (green) in top lane going left, Adversary (red) in bottom lane going right
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color "0,255,0",
    with behavior EgoOncomingBehavior()

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with color "255,0,0",
    with behavior AdvOncomingBehavior()

# @oncoming end: Terminate when vehicles have passed each other and are separating
terminate when (distance from ego to adversary) > globalParameters.PASS_COMPLETE_DIST and \
               (relative position of adversary from ego) @ ego.heading < -globalParameters.PASS_COMPLETE_DIST