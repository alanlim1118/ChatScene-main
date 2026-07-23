"""Scenario Description:

The ego vehicle, represented by a green box, travels along an on-ramp lane, following directly behind a leading vehicle marked by a yellow box as they approach a merge point with a multi-lane highway. To the left, several other vehicles, also indicated by yellow boxes, are established in the lanes of the main highway, moving in the same direction. As the on-ramp runs parallel to the highway, the ego vehicle maintains a following distance behind the lead car, preparing to merge into the traffic flow while coordinating its speed and positioning relative to both the vehicle ahead and the adjacent highway traffic to ensure a safe integration into the lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_COLOR = (0, 255, 0)       # Green box for ego
LEAD_COLOR = (255, 255, 0)    # Yellow box for lead vehicle
HIGHWAY_COLOR = (255, 255, 0) # Yellow box for highway vehicles

param EGO_SPEED = VerifaiRange(8, 12)
param LEAD_SPEED = VerifaiRange(8, 12)
param HIGHWAY_SPEED = VerifaiRange(10, 14)

FOLLOW_DIST = Range(15, 25)
HIGHWAY_VEHICLE_COUNT = Range(3, 5)
MERGE_ZONE_LENGTH = 200

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoMergeBehavior(trajectory, leadCar):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when (distance to leadCar) < 10:
        take SetThrottleAction(0.0)
        take SetBrakeAction(0.5)

behavior LeadVehicleBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=trajectory)

behavior HighwayVehicleBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.HIGHWAY_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a suitable on-ramp that merges into a highway
# Town04 has on-ramps; we look for lanes that have a successor merging maneuver
mergeLanes = []
for lane in network.lanes:
    for maneuver in lane.maneuvers:
        if maneuver.type is ManeuverType.MERGE or maneuver.type is ManeuverType.LANE_CHANGE:
            mergeLanes.append(lane)
            break

require len(mergeLanes) > 0

onRampLane = Uniform(*mergeLanes)

# Get the merge maneuver and target highway lane
mergeManeuver = Uniform(*filter(lambda m: 
    m.type in (ManeuverType.MERGE, ManeuverType.LANE_CHANGE), 
    onRampLane.maneuvers))

highwayLane = mergeManeuver.endLane

# Define trajectories
egoTrajectory = [onRampLane, mergeManeuver.connectingLane, highwayLane]
leadTrajectory = [onRampLane, mergeManeuver.connectingLane, highwayLane]

# Spawn points on the on-ramp
leadSpawnPt = new OrientedPoint on onRampLane.centerline
egoSpawnPt = follow roadDirection from (back of leadSpawnPt) for resample(FOLLOW_DIST)

# Ensure ego is behind lead on the ramp
require (distance from egoSpawnPt to leadSpawnPt) >= 10

# Highway traffic spawn region: along the highway lane near the merge zone
highwayRegion = PointInRegion(highwayLane.centerline)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle (green)
ego = new Car at egoSpawnPt,
    facing roadDirection,
    with color EGO_COLOR,
    with behavior EgoMergeBehavior(egoTrajectory, leadCar)

# Lead vehicle on ramp (yellow)
leadCar = new Car at leadSpawnPt,
    facing roadDirection,
    with color LEAD_COLOR,
    with behavior LeadVehicleBehavior(leadTrajectory)

# Highway vehicles (yellow) placed in the target highway lane
highwayCars = []
lastHwyCar = None
for i in range(HIGHWAY_VEHICLE_COUNT):
    if lastHwyCar is None:
        hwyPos = new OrientedPoint on highwayLane.centerline,
            with regionContainedIn highwayLane
    else:
        hwyPos = follow roadDirection from (front of lastHwyCar) for Range(20, 40),
            with regionContainedIn highwayLane
    
    hwyCar = new Car at hwyPos,
        facing roadDirection,
        with color HIGHWAY_COLOR,
        with behavior HighwayVehicleBehavior([highwayLane]),
        with regionContainedIn highwayLane
    
    highwayCars.append(hwyCar)
    lastHwyCar = hwyCar

# Constraints to ensure valid scenario
require all(car.regionContainedIn is highwayLane for car in highwayCars)
require (distance from ego to leadCar) <= FOLLOW_DIST + 5
require (distance from ego to leadCar) >= 8

# Terminate after sufficient distance traveled or time
terminate when (distance to egoSpawnPt) > MERGE_ZONE_LENGTH + 100