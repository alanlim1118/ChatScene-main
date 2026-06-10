"""Scenario Description:

The ego vehicle is traveling in a straight lane when a pedestrian target emerges from the roadside 
and begins crossing the lane at a constant speed of 5 km/h (approx 1.39 m/s), 
forcing the system to calculate a predicted path and decelerate sufficiently to prevent contact.

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

# 5 km/h is approximately 1.388 m/s
param PED_SPEED = 1.388
param OPT_EGO_SPEED = Range(7, 10)
param OPT_BRAKE_DIST = Range(10, 15)

# Distance for spatial setup
param OPT_GEO_Y_DISTANCE = Range(25, 40)
param OPT_GEO_X_DISTANCE = Range(4, 6)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior PedestrianWalkBehavior(speed):
    do WalkForwardBehavior(speed)

behavior EgoSafetyBehavior(target_speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=brake_dist):
        # Decelerate/Stop to prevent contact
        take SetBrakeAction(1.0)
        take SetThrottleAction(0.0)
        do WaitBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

# Search for a suitable 4-way intersection to provide a long enough straight road section
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)

# Ensure the ego vehicle is going straight
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))

# Define spawn points
egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline

# Target point on the road where the crossing logic is centered
IntSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE

# Pedestrian starts at the roadside (right side)
pedSpawnPt = new OrientedPoint right of IntSpawnPt by globalParameters.OPT_GEO_X_DISTANCE,
    facing IntSpawnPt.heading + 90 deg  # Facing perpendicular to the road to cross

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoSafetyBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DIST)

ped = new Pedestrian at pedSpawnPt,
    with behavior PedestrianWalkBehavior(globalParameters.PED_SPEED)

#################################
# CONSTRAINTS                   #
#################################

# Ensure there is enough distance for the scenario to play out
require 40 <= (distance to intersection) <= 70

# Terminate when the ego vehicle has successfully avoided the pedestrian or time passes
terminate when (distance from ego to IntSpawnPt) < 2 and ego.speed < 0.1
terminate when (distance from ego to IntSpawnPt) > 10 and (distance from ego to egoSpawnPt) > globalParameters.OPT_GEO_Y_DISTANCE