"""Scenario Description:

The ego vehicle is driving alertly on a surface street. 
Suddenly, a pedestrian appears from the side of the road and crosses into the driver's path, 
prompting the ego vehicle to react.

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

# Speed ranges for realistic urban driving
param OPT_EGO_SPEED = Range(5, 10) 
param OPT_ADV_SPEED = Range(1.2, 2.0)

# Distance at which the pedestrian starts the crossing maneuver
param OPT_TRIGGER_DISTANCE = Range(15, 25)

# Safety distance for the ego vehicle to engage brakes
param OPT_BRAKE_DIST = Range(8, 12)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(speed):
    """Ego vehicle drives at a target speed and brakes if a pedestrian is detected in its path."""
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyPedestrians(self, globalParameters.OPT_BRAKE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        terminate

behavior PedestrianCrossingBehavior(target_actor, speed, threshold):
    """Pedestrian waits until the target actor is within range, then crosses the road."""
    # The CrossingBehavior helper dynamically adjusts walking speed to intersect the actor's path
    do CrossingBehavior(target_actor, min_speed=speed, threshold=threshold)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection to provide a standard surface street context
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Choose an incoming lane toward the intersection
egoInitLane = Uniform(*intersection.incomingLanes)

# Define the straight trajectory through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))

# Spawn point for ego
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Calculate a point ahead for the pedestrian to trigger the crossing
# We place it a certain distance along the road from the ego vehicle
triggerPathPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(25, 40)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Ego Vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED)

# Spawn Adversarial Pedestrian
# Positioned to the right of the road, facing perpendicular to the ego's heading
AdvAgent = new Pedestrian right of triggerPathPt by Range(3, 5),
    with heading triggerPathPt.heading + 90 deg,
    with behavior PedestrianCrossingBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_TRIGGER_DISTANCE)

#################################
# CONSTRAINTS                   #
#################################

# Ensure the ego has enough road to drive before reaching the intersection
require 40 <= (distance to intersection) <= 70

# Terminate scenario if the ego has passed the interaction point
terminate when (distance from ego to egoSpawnPt) > 80