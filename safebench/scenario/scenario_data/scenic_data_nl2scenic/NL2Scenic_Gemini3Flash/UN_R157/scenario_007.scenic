"""Scenario Description:

The ego vehicle approaches a section of the road where a motorcycle and a passenger car 
are stopped sequentially within the same lane. The ego vehicle must maintain a safe stop 
to avoid a collision with the stopped vehicles.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'
LEAD_CAR_MODEL = 'vehicle.audi.a2'
MOTORCYCLE_MODEL = 'vehicle.kawasaki.ninja'

param EGO_TARGET_SPEED = Range(7, 12)
param BRAKE_THRESHOLD = Range(12, 18)

WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon', 'WetNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        take SetBrakeAction(1.0)
        wait

behavior EgoSafeStopBehavior(target_speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyObjs(self, brake_dist):
        while True:
            take SetBrakeAction(1.0)
            take SetThrottleAction(0.0)
            wait

#################################
# SPATIAL RELATIONS             #
#################################

# Select a lane on a standard road (not an intersection)
road = Uniform(*filter(lambda r: not any(lane.sections[0].lane.intersection for lane in r.lanes), network.roads))
lane = Uniform(*road.lanes)

# Define spawn points sequentially in the same lane
# We place the leading car first, then the motorcycle behind it, then the ego further back.
lead_car_spawn = new OrientedPoint in lane.centerline

# Motorcycle is stopped 5 to 8 meters behind the lead car
motorcycle_spawn = new OrientedPoint behind lead_car_spawn by Range(5, 8)

# Ego is spawned 30 to 45 meters behind the motorcycle to give it room to approach
ego_spawn = new OrientedPoint behind motorcycle_spawn by Range(30, 45)

#################################
# SCENARIO SPECIFICATION        #
#################################

# The passenger car stopped in the lane
lead_car = new Car at lead_car_spawn,
    with blueprint LEAD_CAR_MODEL,
    with behavior WaitBehavior()

# The motorcycle stopped behind the passenger car
motorcycle = new Motorcycle at motorcycle_spawn,
    with blueprint MOTORCYCLE_MODEL,
    with behavior WaitBehavior()

# The ego vehicle approaching the stopped vehicles
ego = new Car at ego_spawn,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoSafeStopBehavior(globalParameters.EGO_TARGET_SPEED, globalParameters.BRAKE_THRESHOLD)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure the vehicles are placed in a valid part of the lane
require lead_car in lane
require motorcycle in lane
require ego in lane

# Terminate the simulation once the ego has come to a full stop for a period of time
terminate when ego.speed < 0.1 and withinDistanceToAnyObjs(ego, globalParameters.BRAKE_THRESHOLD + 2) for 10 seconds