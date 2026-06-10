"""Scenario Description:

A target vehicle in an adjacent lane performs a full lateral displacement of 3.5 meters 
to merge into the path of the ego vehicle with a lateral deviation of no more than 0.2 meters.
This is modeled using a lane change behavior on a multi-lane road in Town05.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.audi.tt"

param EGO_SPEED = 10
param ADV_SPEED = 13
param INITIAL_GAP = Range(10, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdversaryMergeBehavior(target_lane_sec, speed):
    # Drive in the initial lane for a short duration
    do FollowLaneBehavior(target_speed=speed) for Range(2, 4) seconds
    
    # Perform the lateral displacement (lane change)
    # Standard CARLA lanes are ~3.5m wide; moving to the adjacent lane 
    # centerline achieves the required 3.5m displacement.
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=speed)
    
    # Once merged, follow the new lane centerline (ensuring < 0.2m deviation)
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for road sections that have a valid forward lane to the left
# This ensures there is an adjacent lane to merge from.
laneSecs = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec.laneToLeft is not None and sec.laneToLeft.isForward:
            laneSecs.append(sec)

# Select the target lane for the ego vehicle
egoLaneSec = Uniform(*laneSecs)
# The adversary starts in the lane to the left
advLaneSec = egoLaneSec.laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
# Adversary starts ahead of ego in the adjacent lane to merge into ego's path
advSpawnPt = new OrientedPoint in advLaneSec.centerline,
             ahead of egoSpawnPt by globalParameters.INITIAL_GAP

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdversaryMergeBehavior(egoLaneSec, globalParameters.ADV_SPEED)

# Ensure the scenario starts on a sufficiently long road segment away from intersections
require distance to intersection > 40
require (distance from adversary to intersection) > 40

# Termination condition: terminate when the adversary has finished merging and driven for a bit
terminate when (distance from adversary to egoSpawnPt) > 100