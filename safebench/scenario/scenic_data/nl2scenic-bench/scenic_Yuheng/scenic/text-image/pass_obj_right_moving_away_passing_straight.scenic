"""Scenario Description:

In a top-down view of a four-way intersection featuring gray roads and white dashed lane markings, a blue ego vehicle is stationary in the center lane facing north. A pink adversarial vehicle is driving straight across the intersection from left to right, perpendicular to the ego vehicle's path. The pink car is positioned to the right of the blue car, indicating it is in the process of crossing, while a dashed red line to the left illustrates its trajectory across the junction.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.lincoln.mkz_2017"

param ADV_SPEED = Range(8, 12)

CONST_PERPENDICULAR_DEG = 90 deg
CONST_TOL_DEG = 15 deg
CONST_MIN_PERP_DEG = CONST_PERPENDICULAR_DEG - CONST_TOL_DEG
CONST_MAX_PERP_DEG = CONST_PERPENDICULAR_DEG + CONST_TOL_DEG

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(AdvAgent, 100):
            setClosestTrafficLightStatus(AdvAgent, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior StationaryBehavior():
    while True:
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        wait

behavior AdvCrossingBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego: going straight through intersection (facing north equivalent)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: crossing straight from left to right relative to ego (perpendicular)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color (0, 0, 1),
    with behavior StationaryBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7),
    with behavior AdvCrossingBehavior(advTrajectory)

require monitor TrafficLights()

# Ensure adversary is perpendicular to ego (left-to-right crossing)
require CONST_MIN_PERP_DEG < (egoDir - advDir) < CONST_MAX_PERP_DEG

# Position ego near center of intersection
require 0 <= (distance from egoSpawnPt to intersection) <= 10

# Position adversary already crossing (to the right of ego, partially through intersection)
require -5 <= (distance from advSpawnPt to intersection) <= 15