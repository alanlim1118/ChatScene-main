"""Scenario Description:

The VUT (Vehicle Under Test) travels in a straight line for at least two seconds 
before encountering a stationary bicycle target positioned directly in its driving path 
and facing away. The ego vehicle must then execute an emergency braking maneuver 
or a controlled lateral bypass to avoid a collision.

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
EGO_SPEED = 10  # m/s (~36 km/h)
# Distance to cover in 2 seconds at 10m/s is 20m. 
# We add a buffer so the interaction happens after that time.
BRAKE_THRESHOLD = 15 
INITIAL_DISTANCE = Range(40, 50)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
    # The ego drives forward normally for at least 2 seconds.
    # We use a try-interrupt structure to handle the avoidance logic.
    try:
        do FollowLaneBehavior(target_speed=speed)
    
    # After the initial drive, if it gets within the threshold of the bicycle,
    # it switches to collision avoidance (braking or steering).
    interrupt when withinDistanceToAnyObjs(self, BRAKE_THRESHOLD):
        # DriveAvoidingCollisions handles throttle/brake logic based on the threshold
        do DriveAvoidingCollisions(target_speed=speed, avoidance_threshold=BRAKE_THRESHOLD)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a random lane that is part of a road (not an intersection)
lane = Uniform(*network.lanes)

# Place the bicycle target on the centerline of the lane
bikeSpawnPt = new OrientedPoint on lane.centerline

# Place the ego vehicle behind the bicycle to ensure a straight-line approach
# We ensure it is facing the same direction as the lane (facing away from ego)
egoSpawnPt = new OrientedPoint behind bikeSpawnPt by INITIAL_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Stationary bicycle target facing away from the ego vehicle
target = new Bicycle at bikeSpawnPt,
    with heading bikeSpawnPt.heading,
    with speed 0

# Ego vehicle starts at the spawn point
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)

#################################
# CONSTRAINTS                   #
#################################

# Ensure the setup is on a relatively straight road section
require (distance to intersection) > 60

# Requirement: Ego must be moving toward the target
require (relative heading of target from ego) < 5 deg

# Termination condition: Ego has successfully stopped or passed the obstacle
terminate when (distance to target) < 2 and ego.speed < 0.1
terminate when (ego behavior instances of DriveAvoidingCollisions) and (distance to target) > 10 and (ego ahead of target)