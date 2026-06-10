"""Scenario Description:

The ego vehicle travels at a constant test speed through an intersection while a pedestrian target 
begins crossing from the near side at 5 km/h on a trajectory timed to impact the VUT's 
longitudinal centerline, requiring the system to detect the target at least 4 seconds before 
the predicted collision and intervene to avoid the impact.

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

# Conversion: 5 km/h = 1.388... m/s
PED_SPEED = 1.388
EGO_SPEED = Range(7, 10) # Constant test speed (approx 25-36 km/h)

# Safety parameters
param OPT_BRAKE_THRESHOLD = 12  # Distance to initiate braking
param OPT_DETECTION_DISTANCE = 40 # Ensure visibility for detection time requirement

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    """Ensure the ego has a green light to maintain constant test speed until intervention."""
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed):
    """Ego drives at constant speed until a pedestrian is detected in its path."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToAnyPedestrians(self, globalParameters.OPT_BRAKE_THRESHOLD):
        # Intervention to avoid impact
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        while True:
            wait

behavior PedestrianBehavior(ego_actor, speed):
    """
    Pedestrian crosses the road timed to impact the ego's centerline.
    The CrossingBehavior handles the dynamic steering/speed to intersect the reference actor.
    """
    # Wait until ego is within range to ensure the 4-second detection window is respected
    do CrossingBehavior(reference_actor=ego_actor, min_speed=speed, threshold=globalParameters.OPT_DETECTION_DISTANCE)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a suitable 4-way signalized intersection
intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego moves straight through the intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Identify the pedestrian spawn point on the 'near side' (right sidewalk) 
# relative to the ego's entrance to the intersection
pedCrossing = Uniform(*intersection.crossings)
pedSpawnPt = new OrientedPoint on pedCrossing.sidewalkRegion

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn Ego
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)

# Spawn Pedestrian
# Near side crossing: positioned on the sidewalk near the intersection entry
pedestrian = new Pedestrian at pedSpawnPt,
    with behavior PedestrianBehavior(ego, PED_SPEED)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Requirement: System must be able to detect the target at least 4 seconds before impact.
# At EGO_SPEED, 4 seconds equals distance (EGO_SPEED * 4).
# We ensure the ego starts far enough back to allow for this detection window.
require 45 <= (distance from ego to intersection) <= 60

# Ensure the pedestrian is actually on the right side (near side) relative to ego
require relative heading of (angle to pedestrian) from ego.heading > 0 deg

require monitor TrafficLights()

terminate when (distance from ego to intersection) < 2 and ego.speed < 0.1