"""Scenario Description:

Vehicle A (Ego) is driving and sees Vehicle B approaching in the adjacent lane. 
Vehicle A misjudges the speed and distance of Vehicle B and initiates a lane change into B's lane. 
Vehicle B brakes hard to avoid a collision but is unable to stop in time, striking A from behind.

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

# Models
EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.audi.tt"

# Speeds and Distances
# B is approaching from behind, so ADV_SPEED > EGO_SPEED
param EGO_SPEED = Range(8, 10)
param ADV_SPEED = Range(14, 16)

# The distance at which Ego decides to start the lane change (simulating the misjudgment)
param LC_TRIGGER_DIST = Range(12, 18) 

# Threshold for Adversary to react to the vehicle entering its lane
param BRAKE_THRESHOLD = 15

# Weather
WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon', 'WetNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLaneChangeBehavior(target_lane_sec, speed, trigger_dist):
    """Ego drives then cuts into the adjacent lane when the adversary is at a specific distance."""
    # Drive in current lane until the adversary is within the 'misjudged' distance
    do FollowLaneBehavior(target_speed=speed) until (distance to adv < trigger_dist)
    
    # Perform the lane change
    try:
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=speed)
    interrupt when (withinDistanceToAnyCars(self, 2)): # Emergency brake if collision is imminent during transition
        take SetBrakeAction(1.0)
    
    # Continue driving in the new lane
    do FollowLaneBehavior(target_speed=speed)

behavior AdvBrakeHardBehavior(speed, brake_dist):
    """Adversary drives and slams brakes when a vehicle enters its lane."""
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when (withinDistanceToObjsInLane(self, brake_dist)):
        # Misjudged distance leads to hitting from behind even with full braking
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        # Stay braked
        while True:
            take SetBrakeAction(1.0)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for road sections that have a lane to the left in the same direction
lane_sections_with_left = []
for lane in network.lanes:
    for section in lane.sections:
        if section.laneToLeft and section.laneToLeft.isForward == section.isForward:
            lane_sections_with_left.append(section)

# Select a suitable lane section for the scenario
ego_lane_sec = Uniform(*lane_sections_with_left)
adv_lane_sec = ego_lane_sec.laneToLeft

# Define spawn points: Ego is in front, Adversary is behind in the next lane
ego_spawn_pt = OrientedPoint in ego_lane_sec.centerline
# Spawn adversary 30-40 meters behind to allow for the 'approach'
adv_spawn_pt = OrientedPoint following adv_lane_sec.orientation from ego_spawn_pt offset along (adv_lane_sec.orientation) by -35

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Adversary (Vehicle B)
adv = new Car at adv_spawn_pt,
    with blueprint ADV_MODEL,
    with behavior AdvBrakeHardBehavior(globalParameters.ADV_SPEED, BRAKE_THRESHOLD)

# Spawn Ego (Vehicle A)
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoLaneChangeBehavior(adv_lane_sec, globalParameters.EGO_SPEED, globalParameters.LC_TRIGGER_DIST)

# Ensure they are far enough from intersections to complete the lane change
require distance from ego to ego_lane_sec.road.centerline.lastPoint > 100
require distance from ego to ego_lane_sec.road.centerline.firstPoint > 20

# Terminate scenario after a set time or collision
terminate after 15 seconds