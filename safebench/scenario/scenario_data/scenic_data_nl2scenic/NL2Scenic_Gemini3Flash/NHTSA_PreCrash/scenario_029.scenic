"""Scenario Description:

Vehicle is going straight in an urban area, in daylight, under clear weather conditions, 
at a non-junction location with a posted speed limit of 35 mph; and takes an evasive 
action to avoid an obstacle.

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

# Weather condition as per description
param weather = 'ClearNoon'

# 35 mph is approximately 15.6 m/s
EGO_TARGET_SPEED = 15.6
OBSTACLE_DISTANCE = Range(40, 50)
SAFETY_DISTANCE = 20
BRAKE_INTENSITY = 1.0

# Vehicle models
CAR_MODEL = 'vehicle.audi.a2'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    """Drive at target speed and brake when an obstacle is detected."""
    try:
        # Drive straight following the current lane
        do FollowLaneBehavior(target_speed=EGO_TARGET_SPEED)
    interrupt when withinDistanceToAnyObjs(self, SAFETY_DISTANCE):
        # Evasive action: Emergency braking
        take SetBrakeAction(BRAKE_INTENSITY)
        # Terminate scenario once the vehicle has stopped or avoided the collision
        if self.speed < 0.1:
            terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for lanes that are not part of intersections (non-junction)
# and are in a typical urban setup (Town03)
non_junction_lanes = filter(lambda l: not l.road.intersection, network.lanes)
spawn_lane = Uniform(*non_junction_lanes)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the ego vehicle on a straight non-junction road
ego = new Car on spawn_lane.centerline,
    with rolename 'hero',
    with blueprint CAR_MODEL,
    with behavior EgoBehavior()

# Spawn an obstacle (a static object or a prop) in the path of the ego vehicle
# Using 'Trash' or 'Box' to represent a road obstacle
obstacle = new Trash on spawn_lane.centerline,
    ahead of ego by OBSTACLE_DISTANCE

#################################
# CONSTRAINTS                   #
#################################

# Ensure the ego vehicle has enough room to reach its target speed before encountering the obstacle
require (distance to obstacle) >= 35

# Terminate scenario if the ego vehicle drives too far away (in case it misses the obstacle)
terminate when (distance from ego to obstacle) > 100 and ego.speed > 0