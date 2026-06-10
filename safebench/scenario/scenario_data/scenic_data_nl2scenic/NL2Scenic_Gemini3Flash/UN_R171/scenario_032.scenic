"""Scenario Description:

The ego vehicle travels along a straight path while a bicycle target, initially obstructed 
during its acceleration phase, emerges and crosses perpendicularly at a constant speed of 15 km/h.
The ego vehicle should detect the bicycle target and execute an autonomous braking maneuver 
to avoid a collision.

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
EGO_SPEED = 10
BICYCLE_SPEED = 15 / 3.6  # 15 km/h converted to m/s (~4.17 m/s)

# Distance at which the bicycle starts moving
TRIGGER_DISTANCE = 25 
# Distance threshold for ego to detect and brake
BRAKE_THRESHOLD = 15

WEATHER_OPTIONS = ['ClearNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, brake_dist):
        # Autonomous braking maneuver
        take SetBrakeAction(1.0)
        take SetThrottleAction(0.0)
        while True:
            wait

behavior BicycleBehavior(target_actor, speed, trigger_dist):
    # Wait until the ego vehicle is close enough
    wait until (distance from self to target_actor) < trigger_dist
    
    # Acceleration phase (modeled as a short duration of acceleration)
    do AccelerateForwardBehavior() for 0.5 seconds
    
    # Transition to constant speed crossing
    while True:
        take SetSpeedAction(speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road section
straight_lanes = filter(lambda l: not l.intersection, network.lanes)
lane = Uniform(*straight_lanes)
ego_spawn = new OrientedPoint in lane.centerline

# Define an obstruction point further ahead
obstruction_ref = new OrientedPoint following roadDirection from ego_spawn for 25

# A parked truck to obstruct the ego's view of the bicycle
parked_truck_pt = new OrientedPoint left of obstruction_ref by 3.5,
    facing ego_spawn.heading

# The bicycle starts further to the left, hidden by the truck
bicycle_spawn_pt = new OrientedPoint left of obstruction_ref by 7,
    facing ego_spawn.heading - 90 deg # Crossing perpendicularly (right to left from ego's perspective)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the ego vehicle
ego = new Car at ego_spawn,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED, BRAKE_THRESHOLD)

# Spawn the obstruction (static truck)
truck = new Truck at parked_truck_pt,
    with heading parked_truck_pt.heading

# Spawn the bicycle
bicycle = new Bicycle at bicycle_spawn_pt,
    with behavior BicycleBehavior(ego, BICYCLE_SPEED, TRIGGER_DISTANCE),
    with regionContainedIn None

#################################
# CONSTRAINTS                   #
#################################

# Ensure we are not too close to an intersection for a simple straight path
require (distance to intersection) > 15
require (distance from bicycle to intersection) > 15

# Terminate when the ego has passed the interaction zone
terminate when (distance from ego to ego_spawn) > 60