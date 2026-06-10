"""Scenario Description:

The ego vehicle travels in a straight line for at least two seconds before encountering 
a stationary adult pedestrian target positioned directly in its driving path and facing 
away from the vehicle, requiring the ego vehicle to detect the target and execute an 
autonomous braking maneuver to avoid a collision.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"

# Speed and distance parameters to ensure the 2-second travel requirement
param OPT_EGO_SPEED = Range(5, 8)           # Ego speed in m/s
param OPT_BRAKE_DIST = Range(12, 18)        # Distance at which ego detects and brakes
param OPT_PED_DIST = Range(35, 50)         # Distance to the stationary pedestrian

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior StationaryBehavior():
    # Keep the pedestrian stationary and not moving
    take SetWalkingSpeedAction(0)
    while True:
        wait

behavior EgoBehavior(target_speed, brake_distance):
    try:
        # Drive straight following the lane
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyPedestrians(self, brake_distance):
        # Autonomous braking maneuver
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        # Hold the brake until stopped
        do WaitBehavior() for 5 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a lane that is not part of an intersection to ensure straight travel
lane = Uniform(*filter(lambda l: not l.intersection, network.lanes))

# Define spawn point for the ego vehicle
egoSpawnPt = new OrientedPoint on lane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DIST)

# Stationary adult pedestrian setup
# Positioned directly in path (on centerline) and facing away (same heading as ego)
targetPed = new Pedestrian at egoSpawnPt offset along egoSpawnPt.heading by globalParameters.OPT_PED_DIST,
    with heading egoSpawnPt.heading,
    with behavior StationaryBehavior()

#################################
# CONSTRAINTS                   #
#################################

# Requirement: Travels in a straight line for at least two seconds
# Distance / Speed >= Time
require (distance from ego to targetPed) / globalParameters.OPT_EGO_SPEED >= 2.0

# Ensure there's enough road ahead to perform the maneuver
require distance to intersection > 60

# Ensure the scenario starts with the pedestrian visible in the distance
require ego can see targetPed