"""Scenario Description:

The ego vehicle navigates a curved road section where it must detect a stationary vehicle 
positioned with a 1.0-meter lateral offset from the lane centerline. 
The ego vehicle uses DriveAvoidingCollisions behavior to safely approach the obstacle.

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

EGO_MODEL = "vehicle.tesla.model3"
STATIONARY_MODEL = "vehicle.audi.tt"

param EGO_SPEED = 10
param DISTANCE_TO_STATIONARY = Range(30, 45)
param LATERAL_OFFSET = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryBehavior():
    """Behavior to keep the vehicle stationary with brakes applied."""
    while True:
        take SetBrakeAction(1)
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Select a lane on a road that is likely to have curves (Town04 contains many)
lane = Uniform(*network.lanes)

# Define the starting point for the ego vehicle on the chosen lane
egoSpawnPt = new OrientedPoint on lane.centerline

# Define a point along the centerline for the target vehicle
targetCenterlinePt = new OrientedPoint following lane.centerline from egoSpawnPt for globalParameters.DISTANCE_TO_STATIONARY

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior DriveAvoidingCollisions(target_speed=globalParameters.EGO_SPEED)

# Spawn the stationary vehicle with a lateral offset from the centerline
# We use 'left of' to represent the 1.0m lateral offset
stationary_vehicle = new Car left of targetCenterlinePt by globalParameters.LATERAL_OFFSET,
    with blueprint STATIONARY_MODEL,
    facing targetCenterlinePt.heading,
    with behavior StationaryBehavior()

#################################
# CONSTRAINTS                   #
#################################

# Require that the road section is curved. 
# We define this by ensuring the heading at the ego's start and the target's position 
# differs by at least 5 degrees.
require abs(relative heading of targetCenterlinePt.heading from egoSpawnPt.heading) > 5 deg

# Ensure there is enough distance to the intersection to avoid complex maneuvers 
# unless specified, keeping the focus on the curved road detection.
require (distance to intersection) > 50

# Termination condition: when ego is close to the stationary vehicle or passes it
terminate when (distance to stationary_vehicle) < 5 or (ego ahead of stationary_vehicle)