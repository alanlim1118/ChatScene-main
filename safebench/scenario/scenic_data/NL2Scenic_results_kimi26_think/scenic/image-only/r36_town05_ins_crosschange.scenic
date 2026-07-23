"""Scenario Description:

The ego vehicle approaches from the western road, changes lanes to the right, and executes a right turn to proceed northward onto the road passing directly under the building structure. Simultaneously, two adversary vehicles (blue and yellow) travel straight through the intersection from the southern approach, occupying the lane that continues under the complex, with a red car nearby. The scene takes place at a signalized four-way urban intersection beneath a large building complex, with a parking lot and storage yard to the left and a green park area to the bottom right.

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

param OPT_EGO_SPEED = Range(5, 12)
param OPT_ADV_SPEED = Range(5, 12)

CONST_ADV_OFFSET = 8
CONST_RED_OFFSET = 16

CONST_REL_DEG = -90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_REL_DEG = CONST_REL_DEG - CONST_TOL_DEG
CONST_MAX_REL_DEG = CONST_REL_DEG + CONST_TOL_DEG

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(AdvAgent1, 100):
            setClosestTrafficLightStatus(AdvAgent1, "green")
        if withinDistanceToTrafficLight(AdvAgent2, 100):
            setClosestTrafficLightStatus(AdvAgent2, "green")
        if withinDistanceToTrafficLight(RedCar, 100):
            setClosestTrafficLightStatus(RedCar, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)

behavior AdvBehavior(speed):
    do FollowTrajectoryBehavior(speed, advTrajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego: right turn from western approach
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: straight through from southern approach (conflicting with ego)
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Additional spawn points for the second adversary and the red car
adv2SpawnPt = new OrientedPoint ahead of advSpawnPt by -CONST_ADV_OFFSET
redSpawnPt = new OrientedPoint ahead of advSpawnPt by -CONST_RED_OFFSET

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent1 = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

AdvAgent2 = new Car at adv2SpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

RedCar = new Car at redSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

require monitor TrafficLights()
require CONST_MIN_REL_DEG < (egoDir - advDir) < CONST_MAX_REL_DEG
require 20 <= (distance from egoSpawnPt to intersection) <= 40
require 20 <= (distance from advSpawnPt to intersection) <= 40