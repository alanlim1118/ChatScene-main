"""Scenario Description:

Vehicle is going straight in an urban area, in daylight, under clear weather conditions, 
with a posted speed limit of 25 mph; and then encounters a pedestrian at a non-junction location.

"""

#################################
# MAP AND MODEL                 #
#################################

# Town03 is a large urban map with many junctions and straight roads.
Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# 25 mph is approximately 11.176 m/s
EGO_SPEED = 11.176
EGO_MODEL = "vehicle.lincoln.mkz_2017"

# Weather parameters
param weather = 'ClearNoon'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed):
    try:
        # Drive at the speed limit
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (withinDistanceToAnyPedestrians(self, 15)):
        # Brake if a pedestrian is detected nearby
        take SetThrottleAction(0)
        take SetBrakeAction(1)

behavior PedestrianBehavior(ego_actor):
    # Cross the road dynamically relative to the ego vehicle
    do CrossingBehavior(ego_actor, min_speed=1.5, threshold=20)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lanes that are not part of an intersection (non-junction)
# and ensure they are part of the road network in an urban area.
non_junction_lanes = filter(lambda l: not l.road.intersection and not l.road.isConnectingRoad, network.lanes)
lane = Uniform(*non_junction_lanes)

# Find the sidewalk adjacent to the chosen lane
lane_group = network.laneGroupAt(lane.centerline.start)
sidewalk_region = lane_group.sidewalk

# Ensure we have a valid sidewalk for the pedestrian
require sidewalk_region is not None

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn ego vehicle
ego = new Car on lane.centerline,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)

# Spawn pedestrian on the sidewalk ahead of the ego vehicle
# The pedestrian will be placed on the sidewalk to the right or left of the road
ped = new Pedestrian on sidewalk_region,
    ahead of ego by Range(25, 35),
    with behavior PedestrianBehavior(ego)

#################################
# CONSTRAINTS                   #
#################################

# Ensure the ego vehicle is not starting too close to an intersection
require (distance to intersection) > 30

# Terminate scenario after some time or if ego stops
terminate when ego.speed < 0.1 and (distance to ped) < 10 for 3 seconds