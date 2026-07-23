"""Scenario Description:

In a top-down aerial view of an urban four-way intersection, a red ego vehicle travels straight from left to right along the horizontal road, indicated by a pink trajectory line extending across the junction. Simultaneously, a blue adversarial vehicle approaches from the top on the vertical cross street, proceeding straight downward with a blue trajectory line that cuts perpendicularly across the ego vehicle's path, creating a potential conflict. The surrounding environment includes modern residential buildings and a tree-filled park on the left, while the right side features a paved plaza with a circular fountain and a market area populated with rows of colorful stalls.

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

param OPT_ADV_DISTANCE = Range(60, 70)  # Proximity within which the adversarial car begins to drive
param OPT_BRAKE_DISTANCE = Range(5, 8)  # Distance at which the ego vehicle begins to brake
param OPT_EGO_SPEED = Range (1, 5)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.1,1.2,1.3)

CONST_PERP_DEG = 90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_PERP_DEG = CONST_PERP_DEG - CONST_TOL_DEG
CONST_MAX_PERP_DEG = CONST_PERP_DEG + CONST_TOL_DEG

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

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) 
    interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)  # Ensure no acceleration during braking
        take SetBrakeAction(1)  # Brake to avoid collision

behavior AdvBehavior():
    # Wait until close enough to ego, then proceed straight through the intersection
    do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_DISTANCE
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    terminate
   
#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

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
    with color (1, 0, 0),
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with color (0, 0, 1),
    with behavior AdvBehavior()

require monitor TrafficLights()
require CONST_MIN_PERP_DEG < (egoDir - advDir) < CONST_MAX_PERP_DEG
require 10 <= (distance from advSpawnPt to intersection) <= 20
require 30 <= (distance from egoSpawnPt to intersection) <= 40