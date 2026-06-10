"""Scenario Description:

Driver of Vehicle A (Ego) looked for traffic before changing lanes to the left on a four-lane road. 
The driver did not see Vehicle B (Adversary) in the next lane. 
Vehicle B had no time to react and nowhere to go to avoid Vehicle A.

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

# Use models from the provided list
EGO_MODEL = 'vehicle.tesla.model3'
ADV_MODEL = 'vehicle.audi.tt'

# Speed constants (m/s)
param EGO_SPEED = Range(8, 12)
param ADV_SPEED = Range(12, 15) # Adversary is slightly faster to create a closing-in effect

# Timing for the lane change
LANE_CHANGE_DELAY = Range(1, 3)

WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLaneChangeBehavior(target_lane):
    """Ego drives straight then attempts a lane change to the left."""
    try:
        # Drive straight for a short duration
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) for LANE_CHANGE_DELAY seconds
        
        # Initiate lane change to the left
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane, target_speed=globalParameters.EGO_SPEED)
        
        # Continue driving after change
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, 2):
        # Stop on collision or near-collision
        terminate

behavior AdversaryBehavior():
    """Adversary drives straight, ignoring the collision avoidance to simulate 'no time to react'."""
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Find a road with at least two lanes in the same direction (four-lane road total)
# We filter for roads that have a forward lane group with at least 2 lanes.
roads_with_multiple_lanes = filter(lambda r: r.forwardLanes and len(r.forwardLanes.lanes) >= 2, network.roads)
select_road = Uniform(*roads_with_multiple_lanes)

# 2. Select a section and two adjacent lanes
# laneA is where Ego starts (right), laneB is where Adversary is (left)
select_section = Uniform(*select_road.sections)

# Filter for lane sections that have a lane to their left
eligible_sections = filter(lambda ls: ls.laneToLeft is not None, select_section.forwardLanes.lanes)
laneA_sec = Uniform(*eligible_sections)
laneB_sec = laneA_sec.laneToLeft

# 3. Define spawn points
# We want the adversary to be in the blind spot or close behind ego
ego_spawn_pt = new OrientedPoint on laneA_sec.centerline
# Adversary is behind ego in the adjacent lane
adv_spawn_pt = new OrientedPoint on laneB_sec.centerline, behind ego_spawn_pt by Range(5, 10)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoLaneChangeBehavior(laneB_sec.lane)

adversary = new Car at adv_spawn_pt,
    with blueprint ADV_MODEL,
    with behavior AdversaryBehavior()

# Ensure we are on a relatively long straight stretch for the maneuver
require (distance to ego_spawn_pt) < 100
# Ensure they are in the same road section at the start
require ego.road == adversary.road

# Terminate after some distance
terminate when (distance from ego to ego_spawn_pt) > 80