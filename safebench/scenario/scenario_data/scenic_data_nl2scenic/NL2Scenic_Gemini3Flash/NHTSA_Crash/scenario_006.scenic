"""Scenario Description:

Vehicle B was following Vehicle A too closely. 
Vehicle A had to stop quickly; B could not stop in time and rear-ended A.

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

# Speed in m/s (approx 36 km/h)
TARGET_SPEED = 10 

# Distance between vehicles to represent "following too closely"
FOLLOWING_DISTANCE = Range(5, 8)

# Braking intensity
HARD_BRAKE = 1.0

# Time before the lead car performs the emergency stop (in deciseconds/steps)
BRAKE_TRIGGER_TIME = 40 

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(speed):
    """Vehicle A: Drives for a while then brakes suddenly."""
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when simulation().currentTime > BRAKE_TRIGGER_TIME:
        take SetBrakeAction(HARD_BRAKE)

behavior FollowingVehicleBehavior(speed):
    """Vehicle B: Follows closely and tries to brake but fails to avoid collision."""
    try:
        do FollowLaneBehavior(target_speed=speed)
    
    # Ego detects the car ahead braking or getting too close
    interrupt when withinDistanceToAnyCars(self, 10):
        # Reaction delay to simulate the "could not stop in time" aspect
        wait for Range(0.2, 0.4) seconds
        # Attempt to emergency brake
        while True:
            take SetBrakeAction(HARD_BRAKE)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a random long straight road section
lane = Uniform(*network.lanes)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Vehicle A (Lead Car)
leadCar = new Car on lane.centerline,
    with behavior LeadVehicleBehavior(TARGET_SPEED)

# Spawn Vehicle B (Ego Car) following too closely
ego = new Car following roadDirection from leadCar for -FOLLOWING_DISTANCE,
    with rolename 'hero',
    with behavior FollowingVehicleBehavior(TARGET_SPEED)

#################################
# CONSTRAINTS AND TERMINATION   #
#################################

# Ensure there is enough road ahead for the maneuver
require (distance to intersection) > 50

# Terminate when both cars have basically stopped or a collision is handled
terminate when simulation().currentTime > 100 or (ego.speed < 0.1 and leadCar.speed < 0.1 and simulation().currentTime > BRAKE_TRIGGER_TIME)