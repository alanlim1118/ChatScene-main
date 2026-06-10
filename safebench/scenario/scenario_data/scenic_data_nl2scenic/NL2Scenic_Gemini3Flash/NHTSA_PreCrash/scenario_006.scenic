"""Scenario Description:

Vehicle is going straight in a rural area at night, under clear weather conditions, 
with a posted speed limit of 55 mph or more, and departs the edge of the road 
at a non-junction area.

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

# 55 mph is approximately 24.58 m/s.
param EGO_SPEED = Range(24.6, 30)

# As "Night" is not explicitly in the provided weather list, we select 'ClearSunset' 
# as the closest representative for a clear, low-light environment.
param weather = 'ClearSunset'

EGO_MODEL = 'vehicle.lincoln.mkz_2017'

#################################
# AGENT BEHAVIORS               #
#################################

behavior RoadDepartureBehavior(target_speed):
    """Behavior where the vehicle drives straight and then drifts off the road."""
    # Phase 1: Follow the lane straight at the high speed limit
    try:
        do FollowLaneBehavior(target_speed=target_speed) for Range(5, 10) seconds
    
    # Phase 2: Depart the road by applying a steering offset
    interrupt when True:
        # Choose a direction to steer (left or right) to depart the road edge
        side = Uniform(-1, 1)
        departure_steer = side * Range(0.15, 0.25)
        
        # Continue driving while steering off the road
        while True:
            take SetSteerAction(departure_steer)
            take SetThrottleAction(0.5)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lanes in the rural map that are not associated with any intersection
ruralLanes = filter(lambda l: not l.road.intersections, network.lanes)

# Filter for lanes that provide a long enough stretch for a high-speed scenario
longLanes = filter(lambda l: l.centerline.length > 100, ruralLanes)

# Select a starting lane for the scenario
startLane = Uniform(*longLanes)

# Define a starting point on the chosen lane
spawnPt = new OrientedPoint on startLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the ego vehicle as the primary actor
ego = new Car at spawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior RoadDepartureBehavior(globalParameters.EGO_SPEED)

# Ensure the vehicle starts at a significant distance from any junction
require (distance to intersection) > 50

# Terminate the simulation once the vehicle has significantly departed the road
terminate when (distance from ego to startLane.centerline) > 10