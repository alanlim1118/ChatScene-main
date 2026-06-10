"""Scenario Description:

Vehicle is going straight in an urban area, in daylight, under clear weather conditions, 
with a posted speed limit of 25 mph; and encounters a pedalcyclist at an intersection.

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
EGO_SPEED = 11.17  # 25 mph converted to m/s
BICYCLE_MIN_SPEED = 2
CROSSING_THRESHOLD = 20
SAFETY_DISTANCE = 10

# Weather / Time of Day
param weather = 'ClearNoon'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    """Ego vehicle drives straight through the intersection, braking if the cyclist is in its lane."""
    try:
        do FollowTrajectoryBehavior(target_speed=EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToObjsInLane(self, SAFETY_DISTANCE):
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)

behavior BicycleBehavior(reference_actor):
    """The cyclist crosses the intersection based on the ego's position."""
    do CrossingBehavior(reference_actor, min_speed=BICYCLE_MIN_SPEED, threshold=CROSSING_THRESHOLD)

#################################
# SPATIAL RELATIONS             #
#################################

# 1. Select a 4-way intersection suitable for an urban setting
intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# 2. Define the ego's maneuver: Going straight
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# 3. Define the cyclist's starting point
# We look for a lane entering the same intersection that belongs to a different road (perpendicular)
crossingManeuvers = filter(lambda m: m.startLane.road != egoInitLane.road, intersection.maneuvers)
bikeManeuver = Uniform(*crossingManeuvers)
bikeInitLane = bikeManeuver.startLane

# 4. Create spawn points
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
bikeSpawnPt = new OrientedPoint in bikeInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the Ego Car
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Spawn the Pedalcyclist
bicycle = new Bicycle at bikeSpawnPt,
    with behavior BicycleBehavior(ego)

#################################
# CONSTRAINTS                   #
#################################

# Ensure vehicles start at a reasonable distance from the intersection for the scenario to trigger
require 20 <= (distance from ego to intersection) <= 35
require 15 <= (distance from bicycle to intersection) <= 30

# Terminate when the ego has successfully passed the intersection
terminate when (distance from ego to egoSpawnPt) > 60