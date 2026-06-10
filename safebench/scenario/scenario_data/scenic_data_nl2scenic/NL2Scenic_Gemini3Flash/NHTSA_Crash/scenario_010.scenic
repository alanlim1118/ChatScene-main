"""Scenario Description:

Vehicle A (the adversarial vehicle), in an attempt to pass vehicle B (the ego vehicle), 
cuts around B but too closely. Driver A misjudged the distance between cars and 
clipped the corner of B. This scenario takes place on a multi-lane road.

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

param EGO_SPEED = Range(8, 12)
param ADV_SPEED_Overtake = 16

# Distance at which Vehicle A starts the lane change to overtake
PREPARE_OVERTAKE_DIST = 15

# Distance at which Vehicle A "misjudges" and cuts back in
# A car is roughly 5m long. 7m center-to-center means very little clearance (clipping).
CUT_IN_TRIGGER_DIST = Range(6, 8)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoDrive():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

behavior OvertakeAndClip(target_vehicle):
    # 1. Approach from behind in the same lane
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED + 2) until distance from self to target_vehicle < PREPARE_OVERTAKE_DIST
    
    # 2. Change to the left lane to begin overtaking
    # Identify the lane to the left
    current_ls = self.laneSection
    if current_ls.laneToLeft is None:
        terminate # Scenario fails if no left lane exists
    
    target_lane = current_ls.laneToLeft.lane
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane, target_speed=globalParameters.ADV_SPEED_Overtake)
    
    # 3. Accelerate and pass the target vehicle
    # We continue until we are just slightly ahead of the ego vehicle
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED_Overtake) until (self ahead of target_vehicle) and (distance from self to target_vehicle < CUT_IN_TRIGGER_DIST)
    
    # 4. Cut back into the ego's lane too closely (misjudging distance)
    # The clip occurs because the lane change starts while the vehicles are still overlapping or nearly so
    do LaneChangeBehavior(laneSectionToSwitchTo=target_vehicle.lane, target_speed=globalParameters.ADV_SPEED_Overtake)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a lane that has a lane to its left for overtaking
# Ensure we are not on a single-lane road or in the leftmost lane
lane_filter = filter(lambda l: any(ls.laneToLeft for ls in l.sections) and not l.isOpposite, network.lanes)
starting_lane = Uniform(*lane_filter)

# Define spawn points
ego_spawn_pt = new OrientedPoint in starting_lane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with behavior EgoDrive()

# Vehicle A (the adversary) starts behind Vehicle B (ego)
adversary = new Car behind ego by Range(25, 35),
    with behavior OvertakeAndClip(ego)

# Ensure they are on a relatively straight section to make the maneuver predictable
require (relative heading of adversary from ego) < 10 deg
require (relative heading of ego from adversary) < 10 deg

# Terminate when the maneuver is complete or after a timeout
terminate when (distance from adversary to ego) > 20 and (adversary ahead of ego)
terminate after 40 seconds