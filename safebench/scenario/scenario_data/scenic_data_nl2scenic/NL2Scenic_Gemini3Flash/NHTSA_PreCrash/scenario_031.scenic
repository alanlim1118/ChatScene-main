"""Scenario Description:

Vehicle is going straight in a rural area at night, under clear weather conditions, 
at a non-junction location with a posted speed limit of 55 mph or more; 
and collides with an object on the road.

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

# 55 mph is approximately 24.58 m/s
EGO_SPEED = 24.6 
EGO_MODEL = 'vehicle.lincoln.mkz_2017'

# Based on available options, ClearSunset is the closest approximation to night/low-light 
# that remains "Clear" as requested.
WEATHER_OPTIONS = ['ClearSunset']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed):
    # The vehicle follows the lane at the high speed limit.
    # It does not have collision avoidance logic (DriveAvoidingCollisions), 
    # which ensures it collides with the object as per the description.
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for a non-junction (rural road) location.
# We identify roads that are NOT part of any intersection.
intersection_roads = []
for inter in network.intersections:
    for road in inter.roads:
        intersection_roads.append(road)

# Filter for lanes that are not in intersection roads and have a high speed limit (>= 55 mph).
eligible_lanes = filter(lambda l: l.road not in intersection_roads and (l.speedLimit is None or l.speedLimit >= 24.5), network.lanes)

# Sample a lane from the eligible rural lanes
target_lane = Uniform(*eligible_lanes)

# Define the starting point on the lane
ego_spawn_pt = OrientedPoint on target_lane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the ego vehicle
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with speed EGO_SPEED,
    with behavior EgoBehavior(EGO_SPEED)

# Spawn an object (Box) on the road in the ego's path
# The object is placed on the same lane centerline some distance ahead.
obstacle = new Box on target_lane.centerline,
    ahead of ego by Range(50, 75)

#################################
# REQUIREMENTS AND TERMINATION  #
#################################

# Ensure the ego vehicle starts at a significant distance from the obstacle
require (distance to obstacle) > 40

# Ensure the speed limit of the lane matches the requirement (>= 55 mph)
# If the map doesn't specify speed limits, we proceed with the chosen rural lane.
require target_lane.speedLimit >= 24.5 or target_lane.speedLimit == None

# Terminate when the vehicle has passed the object or a collision is likely to have occurred
terminate when (distance to ego_spawn_pt) > 120