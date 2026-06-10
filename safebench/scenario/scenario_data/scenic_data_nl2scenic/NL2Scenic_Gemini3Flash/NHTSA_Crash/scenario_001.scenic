"""Scenario Description:

A pedestrian crossing a multi-lane roadway was struck by vehicle. 
The driver (ego) was looking for other vehicles and traffic controls, but did not see the pedestrian.
The ego vehicle drives at a steady speed on a multi-lane road and the pedestrian crosses 
perpendicularly using CrossingBehavior, leading to a collision as the ego fails to brake.

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
EGO_MODEL = 'vehicle.audi.tt'

# Speeds and distances
param EGO_SPEED = Range(10, 15)
param PED_SPEED = Range(1.2, 1.6)

# Weather
WEATHER_OPTIONS = ['ClearNoon']
param weather = Uniform(*WEATHER_OPTIONS)

# Threshold for the pedestrian to start crossing relative to ego distance
PED_CROSS_THRESHOLD = 25 

#################################
# AGENT BEHAVIORS               #
#################################

behavior PedestrianBehavior(target_actor):
    # This behavior controls the pedestrian to cross the road 
    # to arrive at the ego's path at the same time as the ego.
    do CrossingBehavior(target_actor, min_speed=globalParameters.PED_SPEED, threshold=PED_CROSS_THRESHOLD)

behavior EgoDriveBehavior(speed):
    # The driver "does not see" the pedestrian, so we use a simple 
    # FollowLaneBehavior without collision avoidance interrupts.
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Filter for multi-lane roads (at least 2 lanes in one direction)
multi_lane_roads = filter(lambda r: any(len(lg.lanes) >= 2 for lg in r.laneGroups), network.roads)
selected_road = Uniform(*multi_lane_roads)

# 2. Select a lane from the forward lanes of that road
selected_lane = Uniform(*selected_road.forwardLanes.lanes)

# 3. Define the spawn point for the Ego vehicle
ego_spawn_pt = new OrientedPoint on selected_lane.centerline

# 4. Identify the sidewalk on the right side of the road to spawn the pedestrian
# We use the lane group of the ego to find the associated sidewalk
ego_lane_group = network.laneGroupAt(ego_spawn_pt)
right_sidewalk = ego_lane_group.sidewalk

# Ensure we have a sidewalk to spawn the pedestrian on
require right_sidewalk is not None

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Ego Vehicle
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoDriveBehavior(globalParameters.EGO_SPEED)

# Spawn Pedestrian on the sidewalk, ahead of the Ego
# The pedestrian is placed roughly 20-30 meters ahead on the right sidewalk
ped = new Pedestrian on right_sidewalk,
    beyond ego_spawn_pt by Range(20, 30),
    with behavior PedestrianBehavior(ego)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure the ego has enough road ahead to perform the scenario
require (distance from ego to selected_lane.centerline[-1]) > 50

# Terminate the scenario if a collision occurs (the "struck" part of the description)
# or if the ego has passed the pedestrian significantly
terminate when (distance to ego_spawn_pt) > 80