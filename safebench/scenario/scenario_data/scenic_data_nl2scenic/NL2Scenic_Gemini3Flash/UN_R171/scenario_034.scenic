"""Scenario Description:

While the ego vehicle travels at its prescribed test speed through an intersection, a bicycle target emerges 
from an obstructed area and crosses perpendicular to the ego vehicle's path at a constant speed of 15 km/h, 
timed precisely so that its centerline aligns with the ego vehicle's impact point, requiring the ego vehicle 
to detect the cyclist and execute an autonomous maneuver to avoid a collision.

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
OBSTRUCTION_MODEL = "vehicle.tesla.cybertruck"

# 15 km/h to m/s is approx 4.17
BICYCLE_SPEED = 4.17

param EGO_SPEED = Range(7, 10)
param BRAKE_THRESHOLD = 10
param CROSSING_THRESHOLD = 20

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.BRAKE_THRESHOLD):
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)
    terminate

behavior BicycleBehavior(target_actor, speed):
    # CrossingBehavior dynamically adjusts speed/timing to meet the target actor
    # We use the fixed speed 4.17 m/s as the min_speed/target_speed
    do CrossingBehavior(target_actor, min_speed=speed, threshold=globalParameters.CROSSING_THRESHOLD)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 4-way intersection for a standard urban crossing
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego goes straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Define the impact point at the center of the intersection
impactPt = egoManeuver.connectingLane.centerline.midpoint

# Find the perpendicular crossing road/lane for the bicycle
# We look for lanes that enter the intersection from the right of the ego's starting lane
egoHeading = egoInitLane.centerline.direction
crossingLane = Uniform(*filter(lambda l: l in intersection.incomingLanes and 
                               abs(relative heading of l.centerline.direction from egoHeading) > 70 deg, 
                               intersection.incomingLanes))

# Spawn point for the bicycle (on the sidewalk/right side of the crossing lane)
bikeSpawnPt = new OrientedPoint on crossingLane.centerline.start,
    facing crossingLane.centerline.direction

# Obstruction: A large truck parked near the corner to block visibility
obstructionPt = new OrientedPoint on egoInitLane.rightEdge.end offset by -5 @ 2,
    facing egoHeading

#################################
# SCENARIO SPECIFICATION        #
#################################

# 1. Spawn the Ego Vehicle
ego = new Car in egoInitLane.centerline,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# 2. Spawn the Obstruction (stationary truck)
obstruction = new Truck at obstructionPt,
    with blueprint OBSTRUCTION_MODEL

# 3. Spawn the Bicycle (adversary)
# Positioned such that it emerges from behind the obstruction
bicycle = new Bicycle at bikeSpawnPt offset by 0 @ 8,
    with behavior BicycleBehavior(ego, BICYCLE_SPEED),
    with regionContainedIn None

#################################
# CONSTRAINTS                   #
#################################

# Ensure ego starts at a reasonable distance to the intersection for the test
require 25 <= (distance from ego to intersection) <= 35

# Ensure the bicycle is far enough to be hidden by the obstruction initially
require (distance from bicycle to impactPt) > 10

# Terminate scenario after the ego has passed the intersection
terminate when (distance from ego to impactPt) > 30 and (ego in egoManeuver.endLane)