"""Scenario Description:

The ego vehicle follows a lead car in the same lane. After a few seconds, 
the lead car suddenly exits the lane by performing a lane change to an 
adjacent lane in the same direction, leaving the lane clear for the ego vehicle.

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

EGO_SPEED = Range(10, 15)
OTHER_SPEED = Range(10, 15)
INITIAL_DISTANCE_APART = Range(10, 15)
TIME_BEFORE_EXIT = Range(3, 6)

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadCarExitBehavior(target_speed, target_lane_sec):
    # Drive normally for a set duration
    do FollowLaneBehavior(target_speed=target_speed) for TIME_BEFORE_EXIT seconds
    
    # Suddenly exit the lane
    try:
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, 5):
        # Safety check during lane change
        take SetBrakeAction(1)
        
    # Continue driving in the new lane
    do FollowLaneBehavior(target_speed=target_speed)

behavior EgoFollowBehavior(target_speed):
    # The ego simply follows its lane
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find all lane sections that have an adjacent lane in the same direction 
# to ensure the lead car has a lane to "exit" into.
lane_sections_with_neighbor = []
for lane in network.lanes:
    for section in lane.sections:
        # Check if there's a lane to the left or right in the same direction
        if section.laneToLeft and section.laneToLeft.isForward == section.isForward:
            lane_sections_with_neighbor.append((section, section.laneToLeft))
        elif section.laneToRight and section.laneToRight.isForward == section.isForward:
            lane_sections_with_neighbor.append((section, section.laneToRight))

# Randomly select a valid starting lane section and its target neighbor
selected_pair = Uniform(*lane_sections_with_neighbor)
start_lane_sec = selected_pair[0]
target_lane_sec = selected_pair[1]

# Define spawn points
other_spawn_pt = new OrientedPoint in start_lane_sec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the Lead Car (the one that will exit)
other = new Car at other_spawn_pt,
    with behavior LeadCarExitBehavior(OTHER_SPEED, target_lane_sec)

# Spawn the Ego Car behind the lead car
ego = new Car following roadDirection from other for -INITIAL_DISTANCE_APART,
    with rolename 'hero',
    with behavior EgoFollowBehavior(EGO_SPEED)

# Ensure they aren't too close to an intersection to avoid turn logic interference
require (distance from other to intersection) > 30
require (distance from other to intersection) < 150

# Terminate scenario after some time to observe the full exit
terminate when simulation().currentTime > 150