"""Scenario Description:

Vehicle is going straight in an urban area, in daylight, under clear weather conditions, 
at an intersection-related location with a posted speed limit of 45 mph; 
and closes in on an accelerating lead vehicle.

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

# 45 mph is approximately 20.1168 m/s
TARGET_SPEED_MS = 20.12
EGO_SPEED = 18.0
LEAD_INITIAL_SPEED = 5.0
LEAD_ACCELERATION_TARGET = 15.0

# Weather/Daylight settings
WEATHER_OPTIONS = ['ClearNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(initial_speed, target_speed):
    """
    Behavior for the lead vehicle to start at a slow speed and 
    gradually accelerate while following the lane.
    """
    current_speed = initial_speed
    # Initially drive at a slow speed
    take SetSpeedAction(current_speed)
    
    # Accelerate gradually until target speed is reached
    while current_speed < target_speed:
        take SetSpeedAction(current_speed)
        current_speed += 0.2  # Simple linear acceleration
        wait
        
    # Once target reached, continue at that speed
    do FollowLaneBehavior(target_speed=target_speed)

behavior EgoBehavior(target_speed):
    """
    Behavior for the ego vehicle to follow the lane at a constant 
    speed higher than the lead's initial speed to 'close in'.
    """
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, 5):
        # Emergency braking if too close (safety measure)
        take SetBrakeAction(1.0)

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Filter for urban intersections and approaching lanes
# Town05 has several 4-way intersections suitable for this scenario.
intersections = filter(lambda i: i.is4Way, network.intersections)
selected_intersection = Uniform(*intersections)

# 2. Select an incoming lane for the ego vehicle
incoming_lane = Uniform(*selected_intersection.incomingLanes)

# 3. Ensure the ego vehicle goes straight through the intersection
straight_maneuvers = filter(lambda m: m.type == ManeuverType.STRAIGHT, incoming_lane.maneuvers)
straight_maneuver = Uniform(*straight_maneuvers)

# Define trajectories for behaviors
full_trajectory = [straight_maneuver.startLane, straight_maneuver.connectingLane, straight_maneuver.endLane]

# Define starting positions
# Lead vehicle is closer to the intersection
# Ego vehicle is further back to allow for the 'closing in' effect
spawn_pt_lead = incoming_lane.centerline[-1]
distance_lead_to_inter = Range(15, 20)
distance_ego_to_lead = Range(15, 25)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the Lead Vehicle
lead_car = new Car following roadDirection from spawn_pt_lead for -distance_lead_to_inter,
    with behavior LeadVehicleBehavior(LEAD_INITIAL_SPEED, LEAD_ACCELERATION_TARGET)

# Spawn the Ego Vehicle behind the Lead Vehicle
ego = new Car following roadDirection from lead_car.position for -distance_ego_to_lead,
    with rolename 'hero',
    with behavior EgoBehavior(EGO_SPEED)

#################################
# CONSTRAINTS                   #
#################################

# Require the road to have a speed limit close to 45 mph (approx 20 m/s)
# Note: CARLA map metadata speed limits are in m/s.
require incoming_lane.road.speedLimit >= 20.0

# Ensure they are on the same lane for the 'closing in' effect
require (distance from ego to lead_car) < 30
require (distance from ego to lead_car) > 10

# Termination condition (optional)
terminate when (distance from ego to selected_intersection) < 2