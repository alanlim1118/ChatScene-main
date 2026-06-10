"""Scenario Description:

Vehicle is going straight and following another lead vehicle in a rural area, in daylight, 
under clear weather conditions, at a non-junction with a posted speed limit of 55 mph 
or more; and the lead vehicle suddenly decelerates.

"""

#################################
# MAP AND MODEL                 #
#################################

# Town07 is a rural environment with narrow roads and few traffic lights, 
# suitable for a rural highway scenario.
Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

# 55 mph is approximately 24.58 m/s
TARGET_SPEED = Range(24.58, 30.0) 
INITIAL_GAP = Range(15, 25)
BRAKE_INTENSITY = 1.0

# Weather/Daylight settings
param weather = 'ClearNoon'

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(speed):
    """The lead vehicle drives at a high speed and then suddenly brakes."""
    try:
        # Drive normally for a random duration
        do FollowLaneBehavior(target_speed=speed) for Range(5, 8) seconds
    interrupt when True:
        # Suddenly decelerate
        while True:
            take SetBrakeAction(BRAKE_INTENSITY)

behavior EgoFollowingBehavior(speed):
    """The ego vehicle follows the lane and avoids collisions."""
    # Using the predefined behavior for collision avoidance
    do DriveAvoidingCollisions(target_speed=speed, avoidance_threshold=15)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter roads to ensure they have a high speed limit and aren't intersections
# Rural roads in Town07 generally fit, but we can filter by metadata if available
high_speed_roads = filter(lambda r: r.speedLimit >= 24 if r.speedLimit else True, network.roads)
select_road = Uniform(*high_speed_roads)
select_lane = Uniform(*select_road.lanes)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the Lead Vehicle first
lead_car = new Car on select_lane.centerline,
    with behavior LeadVehicleBehavior(TARGET_SPEED)

# Spawn the Ego Vehicle behind the Lead Vehicle
ego = new Car following roadDirection from lead_car for -INITIAL_GAP,
    with rolename 'hero',
    with blueprint 'vehicle.lincoln.mkz_2017',
    with behavior EgoFollowingBehavior(TARGET_SPEED)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure we are not at an intersection (non-junction)
require (distance to intersection) > 50

# Safety requirement: Ensure vehicles start on a reasonably long straight road section
require lead_car.laneSection.isForward

# Termination: End simulation once the lead vehicle has stopped or some time after braking
terminate when lead_car.speed < 0.1 for 3 seconds