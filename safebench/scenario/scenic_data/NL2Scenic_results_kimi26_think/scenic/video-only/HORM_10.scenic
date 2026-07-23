"""Scenario Description:

The ego vehicle, represented by a green box, travels along an on-ramp lane, following directly behind a leading vehicle marked by a yellow box as they approach a merge point with a multi-lane highway. To the left, several other vehicles, also indicated by yellow boxes, are established in the lanes of the main highway, moving in the same direction. As the on-ramp runs parallel to the highway, the ego vehicle maintains a following distance behind the lead car, preparing to merge into the traffic flow while coordinating its speed and positioning relative to both the vehicle ahead and the adjacent highway traffic to ensure a safe integration into the lane.

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

param EGO_SPEED = VerifaiRange(8, 12)
param LEAD_SPEED = VerifaiRange(8, 12)
param HIGHWAY_SPEED = VerifaiRange(10, 15)
param FOLLOW_DIST = Range(10, 20)

#################################
# BEHAVIORS                     #
#################################

behavior MergeBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find an on-ramp lane that feeds into a multi-lane highway
onRampLane = None
mergeLane = None

for lane in network.lanes:
    if not lane.isForward:
        continue
    for suc in lane.successorLanes:
        # The successor is on a different road and that road has multiple lanes
        if suc.road is not lane.road and len(suc.road.lanes) >= 2:
            onRampLane = lane
            mergeLane = suc
            break
    if onRampLane is not None:
        break

# The highway road that the on-ramp merges into
highwayRoad = mergeLane.road

# Main highway lanes (forward lanes to the left of the merge lane)
highwayLanes = [l for l in highwayRoad.lanes if l.isForward and l is not mergeLane]

# Use the last section of the on-ramp lane (closest to the merge point)
onRampLaneSec = onRampLane.sections[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Place ego on the on-ramp near the merge point
egoSpawnPt = new OrientedPoint on onRampLaneSec.centerline,
    facing roadDirection,

# Place lead vehicle ahead of ego on the same on-ramp lane
leadSpawnPt = new OrientedPoint at (follow roadDirection from egoSpawnPt for resample(FOLLOW_DIST)),
    facing roadDirection,

# Ego vehicle (green)
ego = new Car at egoSpawnPt,
    with color (0, 1, 0),
    with behavior MergeBehavior(EGO_SPEED),

# Lead vehicle (yellow)
leadVehicle = new Car at leadSpawnPt,
    with color (1, 1, 0),
    with behavior MergeBehavior(LEAD_SPEED),

# Highway traffic: established vehicles in the main highway lanes to the left
highwayVehicles = []
for lane in highwayLanes:
    spawnPt = new OrientedPoint on lane.centerline,
        facing roadDirection,
    v = new Car at spawnPt,
        with color (1, 1, 0),
        with behavior MergeBehavior(HIGHWAY_SPEED),
    highwayVehicles.append(v)

require onRampLane is not None
require mergeLane is not None
require len(highwayLanes) > 0

terminate when (distance from ego to egoSpawnPt) > 200