"""Scenario Description:

The other vehicle suddenly decelerates in front of the ego vehicle.
The ego vehicle must react to the sudden braking of the lead car to avoid a collision.

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

# Selecting models from the provided list
EGO_MODEL = 'vehicle.lincoln.mkz_2017'
LEAD_CAR_MODEL = 'vehicle.audi.a2'

# Speed and braking constants
TARGET_SPEED = 10  # m/s
BRAKE_STRENGTH = 1.0
EGO_BRAKE_THRESHOLD = 12 # Distance at which ego starts braking
DECELERATION_TIME = 50 # Timesteps before lead car brakes (approx 5 seconds)

# Weather setup as per instructions
WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon', 'WetNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadCarBrakingBehavior(speed):
    """Lead car drives at a constant speed and then suddenly brakes."""
    try:
        do FollowLaneBehavior(target_speed=speed)
    
    interrupt when simulation().currentTime > DECELERATION_TIME:
        while True:
            take SetBrakeAction(BRAKE_STRENGTH)

behavior EgoFollowBehavior(speed):
    """Ego car follows the lane and brakes if it gets too close to the lead car."""
    try:
        do FollowLaneBehavior(target_speed=speed)
    
    interrupt when withinDistanceToAnyCars(self, EGO_BRAKE_THRESHOLD):
        take SetBrakeAction(BRAKE_STRENGTH)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a random lane from the network
lane = Uniform(*network.lanes)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the leading car
lead_car = new Car on lane.centerline,
    with blueprint LEAD_CAR_MODEL,
    with behavior LeadCarBrakingBehavior(TARGET_SPEED)

# Spawn the ego vehicle behind the leading car
ego = new Car following roadDirection from lead_car for Range(-15, -10),
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoFollowBehavior(TARGET_SPEED)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure we have enough room to perform the scenario without hitting an intersection immediately
require (distance to intersection) > 80

# Terminate the scenario once the ego car has successfully stopped or enough time has passed
terminate when ego.speed < 0.1 and simulation().currentTime > DECELERATION_TIME + 20