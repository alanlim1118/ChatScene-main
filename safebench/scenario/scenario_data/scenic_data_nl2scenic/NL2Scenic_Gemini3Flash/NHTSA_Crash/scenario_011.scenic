"""Scenario Description:

Driver of Vehicle A (Ego) looked for traffic before changing lanes to the right on a four-lane road. 
The driver did not see Vehicle B (Adversary) in the curb lane. 
Vehicle B braked and steered to avoid Vehicle A.

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

MODEL = 'vehicle.lincoln.mkz_2017'

# Speed constants
param EGO_SPEED = Range(8, 12)
param ADV_SPEED = Range(10, 14)

# Proximity for reaction
param REACTION_DIST = Range(10, 15)

# Weather configuration
WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLaneChangeBehavior(target_speed, target_lane):
    """Ego drives for a short duration and then attempts a lane change to the right."""
    do FollowLaneBehavior(target_speed=target_speed) for Range(3, 5) seconds
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

behavior AdversaryAvoidanceBehavior(target_speed):
    """Adversary drives normally until Ego gets too close, then brakes and steers to avoid."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance to ego < globalParameters.REACTION_DIST):
        # Apply brakes and steer away (to the right, assuming curb lane)
        take SetBrakeAction(1.0)
        take SetSteerAction(0.4)
        do ConstantThrottleBehavior(0) for 2 seconds
        # Return to normal driving if possible or terminate
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Find a road section that has at least two lanes in the same direction (four-lane road total)
# We look for a lane section that has a lane to its right (the curb lane)
lane_sec = Uniform(*filter(lambda ls: ls.isForward and ls.laneToRight is not None, network.laneSections))

ego_lane_sec = lane_sec
adv_lane_sec = lane_sec.laneToRight

# Extract the Lane objects for behavior targeting
ego_lane = ego_lane_sec.lane
adv_lane = adv_lane_sec.lane

# Define spawn points on the centerlines
ego_spawn_pt = OrientedPoint on ego_lane_sec.centerline
adv_spawn_pt = OrientedPoint on adv_lane_sec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoLaneChangeBehavior(globalParameters.EGO_SPEED, adv_lane)

# Adversary starts slightly behind or beside the ego in the curb lane
adversary = new Car at adv_spawn_pt,
    behind ego by Range(5, 12),
    with blueprint MODEL,
    with behavior AdversaryAvoidanceBehavior(globalParameters.ADV_SPEED)

# Ensure they are on a relatively straight part of the road to start
require (distance to ego_spawn_pt) < 100
require (distance from ego to adversary) < 20

# Termination condition
terminate when (distance to ego_spawn_pt) > 150