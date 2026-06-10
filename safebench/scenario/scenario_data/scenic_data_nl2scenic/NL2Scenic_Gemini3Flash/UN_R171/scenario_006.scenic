"""Scenario Description:

The ego vehicle begins a lane change into a center lane at the same time a vehicle from 
the furthest lane starts merging into that same target space, forcing the ego vehicle 
to detect the potential collision risk and either abort the maneuver or adjust its 
path to avoid the other merging vehicle.

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

param EGO_SPEED = Range(7, 10)
param ADV_SPEED = Range(7, 10)

param MERGE_DELAY = Range(1, 2)
param SAFETY_DISTANCE = Range(5, 8)
param INITIAL_GAP = Range(-3, 3) # Relative longitudinal offset between ego and adversary

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_lane_sec, target_speed, adversary_obj):
    # Initial drive
    do FollowLaneBehavior(target_speed=target_speed) until (simulation().currentTime >= globalParameters.MERGE_DELAY)
    
    # Start the lane change
    try:
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=target_speed)
    interrupt when (distance to adversary_obj) < globalParameters.SAFETY_DISTANCE:
        # Avoid collision: Brake and wait
        take SetBrakeAction(1.0)
        do ConstantThrottleBehavior(0) for 3 seconds
        terminate

behavior AdversaryBehavior(target_lane_sec, target_speed):
    # Initial drive
    do FollowLaneBehavior(target_speed=target_speed) until (simulation().currentTime >= globalParameters.MERGE_DELAY)
    
    # Merge into the center lane
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for roads that have at least three lanes in the same direction
# We look for a lane (rightmost) that has a lane to its left, which also has a lane to its left.
laneSecs = []
for lane in network.lanes:
    for sec in lane.sections:
        if (sec.isForward and 
            sec.laneToLeft is not None and sec.laneToLeft.isForward and
            sec.laneToLeft.laneToLeft is not None and sec.laneToLeft.laneToLeft.isForward):
            laneSecs.append(sec)

# Select a suitable starting configuration
egoLaneSec = Uniform(*laneSecs)
centerLaneSec = egoLaneSec.laneToLeft
farLaneSec = centerLaneSec.laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Project ego's position onto the far lane and apply a longitudinal gap
farLaneBasePt = farLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from farLaneBasePt for globalParameters.INITIAL_GAP

#################################
# SCENARIO SPECIFICATION        #
#################################

# The Adversary starts in the furthest lane
adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior(centerLaneSec, globalParameters.ADV_SPEED)

# The Ego starts in the rightmost lane
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(centerLaneSec, globalParameters.EGO_SPEED, adversary)

# Requirements to ensure a valid starting environment
require (distance to intersection) > 50
require (distance from adversary to intersection) > 50

# Termination condition
terminate when (distance to egoSpawnPt) > 100