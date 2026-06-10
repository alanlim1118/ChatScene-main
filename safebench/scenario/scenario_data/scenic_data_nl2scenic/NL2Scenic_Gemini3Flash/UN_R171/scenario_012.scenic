"""Scenario Description:

The ego vehicle stabilizes its lateral position on a straight section before entering a curve where 
it encounters a stationary heavy truck (different category) positioned within the lane. 
The ego vehicle identifies the larger vehicle type and initiates braking to avoid a collision.

"""

#################################
# MAP AND MODEL                 #
#################################

# Town04 is chosen for its highway curves and mountain roads
Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
TRUCK_MODEL = "vehicle.carlamotors.carlacola"

param EGO_SPEED = Range(12, 15)
param BRAKE_THRESHOLD = Range(15, 25) # Distance to start braking for the truck
param BRAKE_STRENGTH = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        take SetBrakeAction(1.0)

behavior EgoBehavior(target_speed, brake_dist):
    # Phase 1: Stabilize lateral position on straight section
    # We simulate "stabilizing" by following the lane for a set duration
    do FollowLaneBehavior(target_speed=target_speed) for Range(4, 6) seconds
    
    # Phase 2: Approach curve and detect stationary truck
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, brake_dist):
        # Identify the larger vehicle (Truck) and initiate braking
        # In this simulation environment, the proximity check acts as detection
        take SetBrakeAction(globalParameters.BRAKE_STRENGTH)
        do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

# Select a lane that leads into a curve. Town04's main loop has significant curvature.
# We filter for a road that has a decent length to allow for stabilization.
lane = Uniform(*network.lanes)

# Place the truck ahead, ideally in a curved section
truck_spawn_pt = new OrientedPoint on lane.centerline

# Place the ego vehicle back on the same lane, allowing for a straight-to-curve transition
ego_spawn_pt = new OrientedPoint following roadDirection from truck_spawn_pt for Range(-60, -80)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Stationary heavy truck
stationary_truck = new Truck at truck_spawn_pt,
    with blueprint TRUCK_MODEL,
    with behavior WaitBehavior()

# Ego vehicle
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.BRAKE_THRESHOLD)

#################################
# CONSTRAINTS                   #
#################################

# Ensure there is enough space to demonstrate the behavior
require (distance from ego to stationary_truck) > 50
# Ensure we are not starting right inside an intersection for this highway scenario
require (distance to intersection) > 20

# Terminate when the vehicle has successfully stopped near the truck
terminate when ego.speed < 0.1 and (distance to stationary_truck) < 15