"""Scenario Description:

In a top-down view of an urban four-way intersection, the ego vehicle is positioned in a lane behind a blue adversarial car, with a blue trajectory line indicating a planned left turn. The lead blue vehicle is situated directly ahead in the same lane and is executing a left turn onto the perpendicular road, while a green adversarial vehicle approaches from the opposite direction in the oncoming lane. The intersection is flanked by a parking lot containing rows of colorful stacked containers on the left and a green space with trees and a building on the right, with a large structure spanning the road at the top of the scene.

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

param OPT_EGO_SPEED = Range(1, 5)
param OPT_LEAD_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(0.8, 0.9, 1.0)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.1, 1.3, 1.5)
param OPT_FOLLOW_DISTANCE = Range(5, 10)

CONST_OPPOSITE_DEG = 180 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_OPPOSITE_DEG = CONST_OPPOSITE_DEG - CONST_TOL_DEG
CONST_MAX_OPPOSITE_DEG = CONST_OPPOSITE_DEG + CONST_TOL_DEG

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(BlueAdv, 100):
            setClosestTrafficLightStatus(BlueAdv, "green")
        if withinDistanceToTrafficLight(GreenAdv, 100):
            setClosestTrafficLightStatus(GreenAdv, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=trajectory)

behavior LeadBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_LEAD_SPEED, trajectory=trajectory)

behavior OncomingBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego and blue lead share the same left-turn maneuver
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane

# Blue lead spawn point ahead in the left-turn lane
blueSpawnPt = new OrientedPoint in egoInitLane.centerline

# Ego spawn point behind the blue lead in the same lane
egoSpawnPt = new OrientedPoint at blueSpawnPt offset by -globalParameters.OPT_FOLLOW_DISTANCE @ 0

# Blue lead and ego follow the same left-turn trajectory
blueTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Green adversarial: straight from opposite direction (conflicting maneuver)
greenManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
greenTrajectory = [greenManeuver.startLane, greenManeuver.connectingLane, greenManeuver.endLane]
greenInitLane = greenManeuver.startLane
greenSpawnPt = new OrientedPoint in greenInitLane.centerline

egoDir = egoSpawnPt.heading
greenDir = greenSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue lead vehicle directly ahead of ego in the same lane
BlueAdv = new Car at blueSpawnPt,
    with heading blueSpawnPt.heading,
    with regionContainedIn None,
    with behavior LeadBehavior(blueTrajectory)

# Ego vehicle behind the blue lead
ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(blueTrajectory)

# Green adversarial approaching from opposite oncoming lane
GreenAdv = new Car at greenSpawnPt,
    with heading greenSpawnPt.heading,
    with regionContainedIn None,
    with behavior OncomingBehavior(greenTrajectory)

require monitor TrafficLights()
# Ensure green is approaching from roughly the opposite direction
require CONST_MIN_OPPOSITE_DEG < abs(egoDir - greenDir) < CONST_MAX_OPPOSITE_DEG
# Ensure blue lead is ahead of ego
require (distance from blueSpawnPt to intersection) < (distance from egoSpawnPt to intersection)
# Distance constraints from intersection
require 10 <= (distance from blueSpawnPt to intersection) <= 30
require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 20 <= (distance from greenSpawnPt to intersection) <= 40