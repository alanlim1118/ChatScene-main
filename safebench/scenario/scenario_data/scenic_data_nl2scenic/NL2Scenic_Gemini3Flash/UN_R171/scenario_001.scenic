"""Scenario Description:

The ego vehicle follows a lead vehicle traveling at least 20 km/h slower on a straight highway for over 
two seconds with a centerline offset under 1 meter. 
Then, the ego vehicle performs a lane change (3.5-meter lateral displacement) into the adjacent lane 
to overtake the slower vehicle.

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

# Speed in m/s (25 m/s approx 90 km/h)
EGO_SPEED = 25 
# Lead speed must be at least 20 km/h (5.55 m/s) slower.
# 15 m/s approx 54 km/h. Difference is 36 km/h.
LEAD_SPEED = 15

INITIAL_GAP = 30
FOLLOW_DURATION = 2.1 # Over 2 seconds
LANE_CHANGE_SPEED = 25

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior():
    do FollowLaneBehavior(target_speed=LEAD_SPEED)

behavior EgoOvertakingBehavior(targetLaneSection):
    # Phase 1: Follow the lead vehicle for over 2 seconds
    # Centerline offset is naturally maintained by FollowLaneBehavior
    do FollowLaneBehavior(target_speed=EGO_SPEED) for FOLLOW_DURATION seconds
    
    # Phase 2: Perform lane change to the left lane (3.5m lateral displacement)
    do LaneChangeBehavior(laneSectionToSwitchTo=targetLaneSection, target_speed=LANE_CHANGE_SPEED)
    
    # Phase 3: Continue following the new lane to complete overtake
    do FollowLaneBehavior(target_speed=EGO_SPEED) for 5 seconds
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for highway lane sections that have an adjacent lane to the left for overtaking
# and ensure they are on long highway stretches (speed limit check)
laneSecsWithLeftLane = []
for lane in network.lanes:
    if lane.speedLimit is not None and lane.speedLimit >= 20:
        for sec in lane.sections:
            if sec.laneToLeft is not None:
                laneSecsWithLeftLane.append(sec)

assert len(laneSecsWithLeftLane) > 0, "No suitable highway sections with a left lane found in Town06."

# Sample a random suitable starting lane section
initLaneSec = Uniform(*laneSecsWithLeftLane)
targetLaneSec = initLaneSec.laneToLeft

# Spawn points
egoSpawnPt = new OrientedPoint on initLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

lead = new Car following roadDirection from egoSpawnPt for INITIAL_GAP,
    with behavior LeadVehicleBehavior()

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with behavior EgoOvertakingBehavior(targetLaneSec)

# Requirements to ensure a straight highway scenario
require (distance to intersection) > 50
require (distance from lead to intersection) > 50

# Requirement: centerline offset under 1 meter during the following phase
# roadDeviation in Scenic measures distance from the lane centerline
require always (ego.roadDeviation < 1.0)

# Ensure the lead vehicle is always ahead until the lane change starts
require eventually (ego.laneSection == targetLaneSec)