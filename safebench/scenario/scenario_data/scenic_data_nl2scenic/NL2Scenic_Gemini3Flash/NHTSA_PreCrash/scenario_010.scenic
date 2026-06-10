"""Scenario Description:

Vehicle is turning right in an urban area, in daylight, under clear weather conditions, 
with a posted speed limit of 25 mph; and encounters a pedalcyclist at an intersection.
The ego vehicle follows a right-turn trajectory while the bicycle crosses the path.

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

# Speed Limit: 25 mph = 11.176 m/s
TARGET_SPEED = 11.176
BIKE_SPEED = 3.0
BRAKE_THRESHOLD = 10
BIKE_THRESHOLD = 20

# Weather: Daylight, clear weather conditions
param weather = 'ClearNoon'

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=TARGET_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, BRAKE_THRESHOLD):
        take SetBrakeAction(1.0)
        take SetThrottleAction(0)

behavior BicycleBehavior(target_actor):
    # The pedalcyclist crosses the road dynamically relative to the ego vehicle
    do CrossingBehavior(target_actor, min_speed=BIKE_SPEED, threshold=BIKE_THRESHOLD)

#################################
# SPATIAL RELATIONS             #
#################################

# Select an urban intersection (3-way or 4-way)
intersec = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))

# Filter for a right turn maneuver
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersec.maneuvers))

# Define the trajectory for the ego vehicle
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Define spawn points
# Ego starts on the incoming lane
egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline

# Bicycle starts at the corner/crosswalk area of the destination lane
# We place it near the start of the endLane, offset to the side to cross it
bikeRefPt = new OrientedPoint at egoManeuver.endLane.centerline.start
bikeSpawnPt = bikeRefPt offset by 6 @ 90 deg 

#################################
# MONITOR                       #
#################################

monitor TrafficLightMonitor:
    # Ensure the encounter is not interrupted by traffic light logic
    freezeTrafficLights()
    setAllIntersectionTrafficLightStatus(intersec, 'green')
    while True:
        wait

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

bicycle = new Bicycle at bikeSpawnPt,
    with heading 90 deg relative to bikeRefPt.heading,
    with behavior BicycleBehavior(ego),
    with regionContainedIn None

# Requirements to ensure the scenario starts at a proper distance for the encounter
require 15 <= (distance to intersec) <= 30
require 5 <= (distance from bicycle to intersec) <= 15

require monitor TrafficLightMonitor()

# Termination condition: Ego has finished the turn or moved past the intersection
terminate when (distance to egoSpawnPt) > 60