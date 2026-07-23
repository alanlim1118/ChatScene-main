"""Scenario Description:

This top-down schematic depicts a two-lane road with traffic flowing in opposite directions. The bottom lane carries traffic moving to the right and contains two pink vehicles, while the top lane carries traffic moving to the left and contains a pink vehicle on the far right. A blue vehicle is positioned in the top lane, which is the lane for oncoming traffic, and is executing a lane change maneuver into the bottom lane. A curved blue arrow illustrates the blue vehicle's trajectory as it merges from the oncoming lane into the gap between a leading pink vehicle ahead and a following pink vehicle behind in the bottom lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

param OPT_SPEED = Range(5, 10)
param GAP_DISTANCE = Range(15, 25)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a forward lane section that has an adjacent opposite-direction lane
candidateLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward:
            if (laneSec._laneToLeft is not None and not laneSec._laneToLeft.isForward) or \
               (laneSec._laneToRight is not None and not laneSec._laneToRight.isForward):
                candidateLaneSecs.append(laneSec)

bottomLaneSec = Uniform(*candidateLaneSecs)

if bottomLaneSec._laneToLeft is not None and not bottomLaneSec._laneToLeft.isForward:
    topLaneSec = bottomLaneSec._laneToLeft
else:
    topLaneSec = bottomLaneSec._laneToRight

# Reference point in the bottom lane where the ego will merge into the gap
refPt = new OrientedPoint in bottomLaneSec.centerline

# Bottom lane: following and leading vehicles relative to the gap
followingPt = follow roadDirection from refPt for -GAP_DISTANCE
leadingPt = follow roadDirection from refPt for GAP_DISTANCE

# Ego (blue vehicle) spawn point in the top lane, aligned with the gap
# It faces the same direction as the bottom lane traffic (wrong-way in the top lane)
egoSpawnPos = topLaneSec.centerline.project(refPt.position)
egoSpawnPt = new OrientedPoint at egoSpawnPos,
    facing refPt.heading

# Top lane pink vehicle on the far right (opposite to top lane's forward direction)
topPinkStart = new OrientedPoint at egoSpawnPos,
    facing (refPt.heading + 180 deg)
topPinkPt = follow roadDirection from topPinkStart for -Range(20, 40)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue vehicle (ego) merging from oncoming top lane into bottom lane
behavior BlueMergeBehavior(target_speed, target_lane):
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
    with regionContainedIn topLaneSec,
    with behavior BlueMergeBehavior(OPT_SPEED, bottomLaneSec)

# Bottom lane following vehicle (pink)
bottomFollowing = new Car at followingPt,
    with regionContainedIn bottomLaneSec,
    with behavior FollowLaneBehavior(target_speed=OPT_SPEED)

# Bottom lane leading vehicle (pink)
bottomLeading = new Car at leadingPt,
    with regionContainedIn bottomLaneSec,
    with behavior FollowLaneBehavior(target_speed=OPT_SPEED)

# Top lane pink vehicle on the far right
topPink = new Car at topPinkPt,
    with regionContainedIn topLaneSec,
    with behavior FollowLaneBehavior(target_speed=OPT_SPEED)

require candidateLaneSecs is not None