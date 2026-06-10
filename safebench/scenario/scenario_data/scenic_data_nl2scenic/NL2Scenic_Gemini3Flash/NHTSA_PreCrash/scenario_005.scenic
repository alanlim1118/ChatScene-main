"""Scenario Description:

Vehicle is turning left/right at an intersection-related location, in a rural area at night, 
under clear weather conditions, with a posted speed limit of 25 mph; 
and then departs the edge of the road.

"""

#################################
# MAP AND MODEL                 #
#################################

# Town07 is a rural environment with narrow roads, corn, and barns, suitable for the description.
Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# 25 mph is approximately 11.176 m/s
TARGET_SPEED = 11.176

# Weather options provided: ClearSunset is chosen to represent the onset of night/darkness 
# under clear conditions as 'Night' is not explicitly in the provided list.
param weather = 'ClearSunset'

# Distance to spawn from the intersection
SPAWN_DISTANCE = Uniform(10, 15)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    # Phase 1: Follow the trajectory through the intersection at the target speed
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=TARGET_SPEED)
    
    # Phase 2: Depart the edge of the road
    # We simulate "departing the road" by applying a steering offset and maintaining some throttle
    # to move towards the shoulder/off-road area.
    try:
        take SetSteerAction(0.3)
        take SetThrottleAction(0.4)
        wait for 5 seconds
    interrupt when withinDistanceToAnyObjs(self, 2):
        take SetBrakeAction(1.0)
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for any intersection in the rural map
intersections = network.intersections
intersec = Uniform(*intersections)

# Select an incoming lane to the intersection
startLane = Uniform(*intersec.incomingLanes)

# Filter for turn maneuvers (Left or Right)
turn_maneuvers = filter(lambda m: m.type in [ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN], startLane.maneuvers)
chosen_maneuver = Uniform(*turn_maneuvers)

# Define the trajectory: start lane -> junction lane -> end lane
ego_trajectory = [chosen_maneuver.startLane, chosen_maneuver.connectingLane, chosen_maneuver.endLane]

# Determine spawn point relative to the intersection
spawn_pt = chosen_maneuver.startLane.centerline[-1]

#################################
# SCENARIO SPECIFICATION        #
#################################

# Choose a standard car model
ego_model = Uniform(*['vehicle.audi.a2', 'vehicle.bmw.grandtourer', 'vehicle.toyota.prius'])

ego = new Car following roadDirection from spawn_pt for -SPAWN_DISTANCE,
    with rolename 'hero',
    with blueprint ego_model,
    with behavior EgoBehavior(ego_trajectory)

# Ensure the vehicle starts on the correct road segment leading to the intersection
require (distance to intersec) <= 20

# Terminate the scenario after the departure behavior completes
terminate when (distance to spawn_pt) > 100