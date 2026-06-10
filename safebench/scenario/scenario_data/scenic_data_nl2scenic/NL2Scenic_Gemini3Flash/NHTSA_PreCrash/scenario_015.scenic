"""Scenario Description:
Vehicle is going straight in an urban area, in daylight, under clear weather conditions, 
at a non-junction with a posted speed limit of 55 mph or more; 
and then drifts into an adjacent vehicle traveling in the same direction.
"""

#################################
# MAP AND MODEL                 #
#################################

# Town05 is a squared-grid urban town with multiple lanes, suitable for high-speed urban driving.
Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# 55 mph is approximately 24.58 m/s.
param EGO_SPEED = Range(25, 30)
param ADV_SPEED = Range(24, 26)

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.audi.etron"

# Weather conditions: Daylight and Clear
param weather = 'ClearNoon'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoDriftBehavior(target_lane_section, speed):
    """Ego drives straight for a while, then drifts (lane changes) into the adjacent vehicle."""
    try:
        # Drive straight initially
        do FollowLaneBehavior(target_speed=speed) for Range(3, 6) seconds
    interrupt when True:
        # Drift into the adjacent lane where the other vehicle is
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_section, target_speed=speed)

behavior AdvBehavior(speed):
    """Adjacent vehicle maintains speed in its lane."""
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Find a road section that is not at an intersection and has at least two parallel lanes in the same direction.
# We filter for lane sections that have a lane to their left and are not part of an intersection.
suitableSections = filter(lambda ls: ls.laneToLeft is not None 
                          and ls.lane.intersection is None 
                          and not ls.lane.road.isConnectingRoad, network.laneSections)

# Select a random starting lane section for the ego
ego_ls = Uniform(*suitableSections)
# Identify the adjacent lane section to the left
adv_ls = ego_ls.laneToLeft

# Define starting positions
# ego_ls.lane.centerline gives the path. We'll pick a point at the start of a straight stretch.
ego_start_pt = ego_ls.lane.centerline[0]
adv_start_pt = adv_ls.lane.centerline[0]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the adversary car in the adjacent lane
adversary = new Car at adv_start_pt,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior(globalParameters.ADV_SPEED)

# Spawn the ego car in its lane, slightly behind or level with the adversary
ego = new Car at ego_start_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoDriftBehavior(adv_ls, globalParameters.EGO_SPEED)

# Ensure they start near each other to make the drift relevant
require (distance from ego to adversary) < 10
# Ensure we are on a long enough road to perform the maneuver
require ego_ls.lane.length > 100

# Termination condition (optional): stop when the drift/lane change is nearly complete or a collision occurs
terminate when (distance from ego to adversary) < 1.5 or (ego.laneSection == adv_ls)