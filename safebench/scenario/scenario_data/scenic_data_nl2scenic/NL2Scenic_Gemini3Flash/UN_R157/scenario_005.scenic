"""Scenario Description:

The ego vehicle encounters a stationary powered two-wheeler positioned directly in the center of the lane 
when ego vehicle is navigating a highway curve. The ego vehicle must detect the object despite the 
lateral offset caused by the curve and execute an emergency braking maneuver to avoid a collision.

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

# Ego configuration
EGO_MODEL = "vehicle.lincoln.mkz_2017"
EGO_SPEED = Range(20, 25)  # Typical highway speed in m/s
EGO_BRAKE_THRESHOLD = 30   # Distance in meters to trigger emergency braking

# Obstacle configuration
motorcycleModels = ['vehicle.harley-davidson.low_rider', 'vehicle.kawasaki.ninja', 'vehicle.yamaha.yzf']
MOT_MODEL = Uniform(*motorcycleModels)

# Weather configuration
WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon', 'ClearSunset']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(target_speed, brake_threshold):
    """Drive at highway speeds and perform emergency braking upon detecting an object in the lane."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    
    interrupt when withinDistanceToObjsInLane(self, brake_threshold):
        # Perform emergency braking
        take SetBrakeAction(1.0)
        take SetHandBrakeAction(True)
        # Stay stopped for a few seconds before terminating
        do WaitBehavior() for 5 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for highway lanes in Town04 (usually characterized by higher speed limits)
highwayLanes = filter(lambda l: l.speedLimit > 20, network.lanes)
selectedLane = Uniform(*highwayLanes)

# Pick a spot for the stationary motorcycle on the centerline of the highway
targetSpawnPt = new OrientedPoint on selectedLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the stationary motorcycle (the obstacle)
stationaryMotorcycle = new Motorcycle at targetSpawnPt,
    with blueprint MOT_MODEL,
    with behavior WaitBehavior()

# Spawn the Ego vehicle behind the motorcycle
# We place it far enough back to allow it to reach target speed and encounter the curve
ego = new Car following roadDirection from targetSpawnPt for Range(-80, -60),
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED, EGO_BRAKE_THRESHOLD)

#################################
# CONSTRAINTS AND TERMINATION   #
#################################

# Ensure the scenario happens on a curved section of the highway
# We check the relative heading between the ego and the target to ensure the road has turned
require 5 deg < abs(relative heading of stationaryMotorcycle.heading from ego.heading) < 45 deg

# Ensure we are not too close to an intersection to maintain the 'highway' feel
require (distance from ego to intersection) > 50

# Terminate when the ego has successfully stopped near the obstacle
terminate when ego.speed < 0.1 and (distance to stationaryMotorcycle) < EGO_BRAKE_THRESHOLD