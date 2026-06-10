"""Scenario Description:

The ego vehicle approaches the stationary pedestrian at a high test velocity, 
requiring the ego vehicle to detect the target and execute an autonomous 
braking maneuver to avoid a collision.

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

# Test velocity: high speed (m/s)
param EGO_SPEED = Range(15, 20) 

# Threshold distance to trigger the autonomous braking maneuver
param BRAKE_THRESHOLD = Range(20, 30)

# Full braking amount
BRAKE_AMOUNT = 1.0

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryBehavior():
    while True:
        wait

behavior EgoAEBBehavior(speed, threshold):
    try:
        # Drive at the high test velocity
        do FollowLaneBehavior(target_speed=speed)
    
    interrupt when withinDistanceToAnyPedestrians(self, threshold):
        # Execute autonomous braking maneuver
        while True:
            take SetBrakeAction(BRAKE_AMOUNT)
            if self.speed < 0.1:
                terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a random lane from the road network
lane = Uniform(*network.lanes)

# Place the pedestrian at a point on the lane's centerline
pedestrian_spawn_pt = new OrientedPoint on lane.centerline

# Place the ego vehicle behind the pedestrian to allow for acceleration/approach
ego_spawn_pt = new OrientedPoint following roadDirection from pedestrian_spawn_pt for Range(-80, -60)

#################################
# SCENARIO SPECIFICATION        #
#################################

# The stationary pedestrian target
target = new Pedestrian at pedestrian_spawn_pt,
    with behavior StationaryBehavior()

# The ego vehicle performing the test
ego = new Car at ego_spawn_pt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoAEBBehavior(globalParameters.EGO_SPEED, globalParameters.BRAKE_THRESHOLD)

#################################
# REQUIREMENTS                  #
#################################

# Ensure the scenario happens on a straight road section away from intersections
require (distance to intersection) > 20
require (distance from ego to target) > 50

# Terminate scenario when ego has successfully stopped or if a collision occurs
terminate when ego.speed < 0.1 and (distance from ego to target) < 15