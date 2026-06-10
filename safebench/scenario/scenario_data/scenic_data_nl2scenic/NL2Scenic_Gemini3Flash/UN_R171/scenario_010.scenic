"""Scenario Description:

While operating in a non-highway environment (Town01), the ego vehicle rounds a curve to find a stationary vehicle 
facing toward the VUT (oncoming orientation) in its lane. The ego identifies the front of the vehicle 
as a collision hazard and responds by braking to a stop.

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
BLOCKER_MODEL = "vehicle.audi.tt"

param OPT_EGO_SPEED = Range(5, 8)
param OPT_BRAKE_THRESHOLD = Range(12, 18)
param OPT_SPAWN_DIST = Range(35, 50)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, threshold):
    """Ego drives along the lane and brakes when the stationary vehicle is detected."""
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyCars(self, threshold):
        # Apply full brakes to avoid collision
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        # Stay stopped and terminate the scenario
        do ConstantThrottleBehavior(0) for 5 seconds
        terminate

behavior StationaryBehavior():
    """Behavior for the parked vehicle to remain stationary."""
    while True:
        take SetThrottleAction(0)
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a lane that is not part of an intersection
lane = Uniform(*filter(lambda l: not l.intersection, network.lanes))

# Define the starting point for the ego vehicle
egoSpawnPt = new OrientedPoint in lane.centerline

# Define the point for the stationary vehicle ahead in the same lane
blockerSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_SPAWN_DIST

# Ensure there is a curve between the ego starting point and the blocker
# This is measured by the change in lane heading over the distance
curve_angle = abs(relative heading of blocker_spawn.heading from ego_spawn.heading)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the Ego Vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_THRESHOLD)

# Spawn the Stationary Blocker Vehicle
# It is placed in the same lane but facing the ego (180 degree rotation from lane direction)
blocker = new Car at blockerSpawnPt,
    with blueprint BLOCKER_MODEL,
    with heading blockerSpawnPt.heading + 180 deg,
    with behavior StationaryBehavior()

#################################
# CONSTRAINTS                   #
#################################

# Ensure the configuration represents 'rounding a curve'
require 10 deg < curve_angle < 45 deg

# Ensure we are not too close to intersections to avoid traffic logic interference
require distance to intersection > 15
require (distance from ego to blocker) > 30

# Terminate if the ego successfully stops or after a reasonable time
terminate when ego.speed < 0.1 and withinDistanceToAnyCars(ego, globalParameters.OPT_BRAKE_THRESHOLD + 2)