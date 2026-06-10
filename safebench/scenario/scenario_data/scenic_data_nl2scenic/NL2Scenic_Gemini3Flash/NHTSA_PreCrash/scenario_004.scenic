"""Scenario Description:

Vehicle is going straight in a rural area, in daylight, under clear weather conditions, 
with a posted speed limit of 35 mph or less; and runs a stop sign at an intersection.
Town07 is used as it represents a rural environment.

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

# 35 mph is approximately 15.65 m/s
MAX_SPEED_LIMIT = 15.65

# Weather and Time of Day
WEATHER_OPTIONS = ['ClearNoon']
param weather = Uniform(*WEATHER_OPTIONS)

# Speed for the ego vehicle (running the stop sign)
# We choose a speed close to the limit to simulate normal driving that doesn't slow down
EGO_TARGET_SPEED = Range(10, 14) 
SPAWN_DISTANCE_BACK = Range(25, 35)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, trajectory):
    """
    Ego vehicle follows the lane trajectory at a constant speed.
    By using FollowTrajectoryBehavior without explicit stop conditions,
    the vehicle will naturally ignore (run) the stop sign at the intersection.
    """
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for all intersections in the rural map
intersections = filter(lambda i: i.is4Way or i.is3Way, network.intersections)
intersec = Uniform(*intersections)

# Select a starting lane that allows the vehicle to go straight through the intersection
startLane = Uniform(*filter(lambda l: any(m.type == ManeuverType.STRAIGHT for m in l.maneuvers), intersec.incomingLanes))

# Select the specific straight maneuver and build the trajectory
straight_maneuver = Uniform(*filter(lambda m: m.type == ManeuverType.STRAIGHT, startLane.maneuvers))
ego_trajectory = [straight_maneuver.startLane, straight_maneuver.connectingLane, straight_maneuver.endLane]

# Define the point where the lane meets the intersection
intersection_entry_pt = startLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Requirement: Posted speed limit must be 35 mph (15.65 m/s) or less
require startLane.speedLimit <= MAX_SPEED_LIMIT

# Spawn the ego vehicle at a distance behind the intersection entry point
ego = new Car following roadDirection from intersection_entry_pt for -SPAWN_DISTANCE_BACK,
    with rolename 'hero',
    with behavior EgoBehavior(EGO_TARGET_SPEED, ego_trajectory)

# Termination: End scenario once the vehicle has successfully crossed the intersection
terminate when (distance from ego to intersection_entry_pt) > 50