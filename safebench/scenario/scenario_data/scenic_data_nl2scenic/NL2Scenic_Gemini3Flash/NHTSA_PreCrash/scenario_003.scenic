"""Scenario Description:

Vehicle is going straight in an urban area, in daylight, under clear weather conditions, with a 
posted speed limit of 35 mph; vehicle then runs a red light, crossing an intersection and 
colliding with another vehicle crossing the intersection from a lateral direction.

The ego vehicle (victim) proceeds through the intersection on a green light, while the 
adversarial vehicle (violator) ignores a red light from a lateral direction, resulting in a collision.

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

# 35 mph is approximately 15.64 m/s
SPEED_LIMIT = 15.64

# Weather and time of day
param weather = 'ClearNoon'

# Distances to the intersection center to ensure simultaneous arrival
SPAWN_DISTANCE = 35 

#################################
# MONITORS                      #
#################################

monitor TrafficLightControl():
    """
    Freezes traffic lights and ensures the Ego has a green light 
    while the Adversary has a red light to simulate 'running a red light'.
    """
    freezeTrafficLights()
    while True:
        # Set ego's approaching light to green
        setClosestTrafficLightStatus(ego, "green", distance=100)
        # Set adversary's approaching light to red
        setClosestTrafficLightStatus(adversary, "red", distance=100)
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior DrivingBehavior(target_speed):
    """
    Standard driving behavior to follow the lane at a constant speed.
    In this scenario, we use FollowLaneBehavior which, by default, 
    will not check for traffic light states unless explicitly told, 
    allowing the adversary to 'run' the red light.
    """
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Filter for a 4-way signalized intersection in the urban environment
intersections = filter(lambda i: i.is4Way and i.isSignalized, network.intersections)
inter = Uniform(*intersections)

# 2. Define the Ego's path (going straight)
egoManeuver = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT, inter.maneuvers))
egoInitLane = egoManeuver.startLane

# 3. Define the Adversary's path (going straight from a lateral/conflicting direction)
# Conflicting maneuvers from a lateral direction are part of the intersection maneuvers
advManeuver = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT 
                             and m in egoManeuver.conflictingManeuvers, inter.maneuvers))
advInitLane = advManeuver.startLane

# 4. Define spawn points at the specified distance from the intersection
egoSpawnPt = OrientedPoint on egoInitLane.centerline
advSpawnPt = OrientedPoint on advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the Ego vehicle (the one being hit)
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with behavior DrivingBehavior(target_speed=SPEED_LIMIT)

# Spawn the Adversary vehicle (the one running the red light)
adversary = new Car at advSpawnPt,
    with behavior DrivingBehavior(target_speed=SPEED_LIMIT)

#################################
# CONSTRAINTS                   #
#################################

# Ensure vehicles are spawned at the correct distance to facilitate the collision
require 34 <= (distance from ego to inter) <= 36
require 34 <= (distance from adversary to inter) <= 36

# Ensure they are coming from different directions (lateral)
require 80 deg <= abs(relative heading of adversary from ego) <= 100 deg

# Activate traffic light control
require monitor TrafficLightControl()

# Optional: Terminate simulation after a set time or collision
terminate after 10 seconds