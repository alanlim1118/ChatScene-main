"""Scenario Description:

The ego vehicle approaches an intersection in a straight line while a vehicle target enters 
from the side at a speed of up to 60 km/h, requiring the ego vehicle to recognize the 
crossing vehicle and decelerate or stop to provide the right of way and avoid colliding 
with the target's side.

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

# Speed of 60 km/h is approximately 16.6 m/s
param OPT_ADV_SPEED = Range(12, 16.6)
param OPT_EGO_SPEED = Range(8, 12)

# Trigger distance: when the ego is this close, the adversary begins its crossing
param OPT_TRIGGER_DISTANCE = Range(30, 45)

# Braking threshold for ego
param OPT_BRAKE_THRESHOLD = Range(10, 15)

# Perpendicular heading constants
CONST_RIGHT_DEG = -90 deg
CONST_LEFT_DEG = 90 deg
CONST_TOL_DEG = 15 deg

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    """
    Monitor to ensure the ego has a green light to proceed, 
    while the adversary might be running a red or simply crossing.
    """
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(AdvAgent, 100):
            # In this scenario, we set the adversary's light to red to simulate 
            # them running the light or forcing right of way.
            setClosestTrafficLightStatus(AdvAgent, "red")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(target_speed, brake_dist):
    """
    Ego drives straight and monitors for crossing traffic.
    """
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyCars(self, brake_dist):
        # Recognize crossing vehicle and stop
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 2 seconds
        # After a brief stop, try to resume slowly or stay stopped
        do FollowLaneBehavior(target_speed=target_speed/2)

behavior AdvBehavior(actor_reference, target_speed, trigger_distance):
    """
    Adversary waits for ego to approach, then crosses the intersection.
    """
    # Wait until ego is close enough to create a conflict
    do WaitBehavior() until (distance from self to actor_reference) <= trigger_distance
    
    # Drive at the specified speed (up to 60 km/h)
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a 4-way signalized intersection
intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego moves straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary moves straight from a conflicting lane (left or right)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoHeading = egoSpawnPt.heading
advHeading = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_THRESHOLD)

AdvAgent = new Car at advSpawnPt,
    with behavior AdvBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_TRIGGER_DISTANCE)

# Ensure the vehicles are indeed perpendicular to simulate a side-crossing
require (CONST_LEFT_DEG - CONST_TOL_DEG < (egoHeading - advHeading) < CONST_LEFT_DEG + CONST_TOL_DEG) or \
        (CONST_RIGHT_DEG - CONST_TOL_DEG < (egoHeading - advHeading) < CONST_RIGHT_DEG + CONST_TOL_DEG)

# Start vehicles at appropriate distances from the intersection
require 35 <= (distance from egoSpawnPt to intersection) <= 50
require 10 <= (distance from advSpawnPt to intersection) <= 20

# Activate traffic light control
require monitor TrafficLights()

# Terminate when ego has passed the intersection or a collision is imminent (scenic default)
terminate when (distance from ego to intersection) > 50 and (distance from AdvAgent to intersection) > 50