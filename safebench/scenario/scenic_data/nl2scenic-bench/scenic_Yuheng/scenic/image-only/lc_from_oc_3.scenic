"""Scenario Description:

This top-down schematic depicts a two-lane road with traffic flowing in opposite directions, indicated by pink vehicles and directional arrows. The bottom lane carries traffic moving to the right and contains two pink vehicles, while the top lane carries traffic moving to the left and contains a pink vehicle on the far right. A blue vehicle is positioned in the top lane, which is the lane for oncoming traffic, and is executing a lane change maneuver into the bottom lane. A curved blue arrow illustrates the blue vehicle's trajectory as it merges from the oncoming lane into the gap between a leading pink vehicle ahead and a following pink vehicle behind in the bottom lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"  # Blue vehicle
NPC_MODEL = "vehicle.tesla.model3"      # Pink vehicles

param OPT_EGO_SPEED = Range(6, 9)
param OPT_NPC_SPEED = Range(5, 8)
param GAP_DISTANCE = Range(25, 40)       # Gap between leading and following NPC in bottom lane
param LEADING_DIST = Range(30, 50)       # Distance of leading NPC ahead of merge point
param FOLLOWING_DIST = Range(20, 35)     # Distance of following NPC behind merge point
param ONCOMING_OFFSET = Range(40, 60)    # How far ahead in oncoming lane the ego starts

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLaneChangeBehavior(target_speed, target_lane):
    """Ego starts in oncoming lane then changes into the bottom (forward) lane."""
    try:
        do FollowLaneBehavior(target_speed=target_speed) until (distance from self to LeadingNPC < globalParameters.LEADING_DIST + 10)
        do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to FollowingNPC < 5):
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Find forward lane sections that have an adjacent opposite-direction lane to the left
# In CARLA, typically the left lane of a forward road is the oncoming (backward) lane
candidateLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and not laneSec._laneToLeft.isForward:
            candidateLaneSecs.append(laneSec)

require len(candidateLaneSecs) > 0

bottomLaneSec = Uniform(*candidateLaneSecs)
topLaneSec = bottomLaneSec._laneToLeft  # Oncoming (opposite direction) lane

# Define a reference point on the bottom lane where the merge gap will be centered
mergeRefPt = new OrientedPoint on bottomLaneSec.centerline

# Leading NPC position: ahead of merge reference in bottom lane
leadingPos = follow roadDirection from mergeRefPt for resample(globalParameters.LEADING_DIST)
LeadingSpawnPt = new OrientedPoint at leadingPos, facing roadDirection

# Following NPC position: behind merge reference in bottom lane
followingPos = follow roadDirection from mergeRefPt for -resample(globalParameters.FOLLOWING_DIST)
FollowingSpawnPt = new OrientedPoint at followingPos, facing roadDirection

# Far-right NPC in top (oncoming) lane: placed far along the oncoming lane
# Since top lane is backward, "far right" in schematic means far along its travel direction
oncomingFarPos = follow roadDirection from (topLaneSec.centerline.project(mergeRefPt.position)) for -resample(Range(60, 90))
OncomingNPCSpawnPt = new OrientedPoint at oncomingFarPos, facing roadDirection

# Ego spawn: in the top (oncoming) lane, positioned so it can merge into the gap
egoOncomingPos = follow roadDirection from (topLaneSec.centerline.project(mergeRefPt.position)) for -resample(globalParameters.ONCOMING_OFFSET)
EgoSpawnPt = new OrientedPoint at egoOncomingPos, facing roadDirection

#################################
# SCENARIO SPECIFICATION        #
#################################

# Leading pink vehicle in bottom lane (ahead of merge gap)
LeadingNPC = new Car at LeadingSpawnPt,
    with regionContainedIn bottomLaneSec,
    with blueprint NPC_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_NPC_SPEED)

# Following pink vehicle in bottom lane (behind merge gap)
FollowingNPC = new Car at FollowingSpawnPt,
    with regionContainedIn bottomLaneSec,
    with blueprint NPC_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_NPC_SPEED)

# Pink vehicle in top (oncoming) lane, far right
OncomingNPC = new Car at OncomingNPCSpawnPt,
    with regionContainedIn topLaneSec,
    with blueprint NPC_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_NPC_SPEED)

# Blue ego vehicle in top (oncoming) lane, executing lane change into bottom lane gap
ego = new Car at EgoSpawnPt,
    with regionContainedIn topLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoLaneChangeBehavior(
        globalParameters.OPT_EGO_SPEED,
        bottomLaneSec
    )

# Ensure sufficient road length for the scenario
require distance to intersection >= 100