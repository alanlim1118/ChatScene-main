"""Scenario Description:

The ego vehicle travels at a constant speed in a straight line for at least two seconds 
with a lateral offset of less than 0.5 meters before approaching a stationary vehicle 
positioned directly ahead in the same lane.

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

EGO_SPEED = 10  # m/s
# At 10m/s, 2 seconds is 20 meters. We set the distance to the stationary vehicle 
# further away to ensure it drives for at least 2 seconds.
DISTANCE_TO_STATIONARY = Range(40, 60) 
LATERAL_OFFSET = Range(-0.4, 0.4)
MODEL = 'vehicle.tesla.model3'

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryBehavior():
    # Maintain zero speed
    while True:
        take SetSpeedAction(0)

behavior EgoDriveBehavior(speed):
    # Drive at a target speed following the lane
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a lane that is long enough and not inside an intersection
lanes = filter(lambda l: l.centerline.length > 100, network.lanes)
initLane = Uniform(*lanes)

# Define a starting point for the ego vehicle
# Ensure we are not too close to the end of the road
egoSpawnPt = new OrientedPoint on initLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the stationary vehicle ahead in the same lane
adversary = new Car following roadDirection from egoSpawnPt for DISTANCE_TO_STATIONARY,
    with blueprint MODEL,
    with behavior StationaryBehavior()

# Spawn the ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoDriveBehavior(EGO_SPEED),
    with roadDeviation LATERAL_OFFSET

#################################
# CONSTRAINTS                   #
#################################

# Ensure the ego and adversary are in the same lane
require ego.lane == adversary.lane

# Ensure the path is relatively straight by requiring distance from intersections
require (distance to intersection) > 70
require (distance from adversary to intersection) > 10

# Ensure the ego has enough room to travel for 2 seconds (20m) before reaching the stationary car
require (distance from ego to adversary) >= 25

# Enforce the lateral offset requirement throughout the simulation
require always (ego.roadDeviation < 0.5 and ego.roadDeviation > -0.5)

# Terminate when ego gets very close to the stationary vehicle
terminate when distance from ego to adversary < 5