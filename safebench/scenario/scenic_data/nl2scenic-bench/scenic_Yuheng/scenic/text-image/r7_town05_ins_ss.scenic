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
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.0, 1.1, 1.2)
param OPT_ADV_DISTANCE = Range(55, 70)  # Distance at which adv starts moving
param OPT_BRAKE_DISTANCE = Range(5, 9)  # Ego braking trigger distance

# Adversary approaches from top (north), ego goes left-to-right (east)
# In CARLA/Scenic heading convention: 0 deg = East, 90 deg = South
# So ego heading ~0 deg, adv heading ~90 deg => difference ~ -90 deg
CONST_PERP_DEG = -90 deg
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
        do FollowTrajectoryBehavior(globalParameters.OPT_EGO_SPEED, egoTrajectory)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        abort
    terminate

behavior AdvBehavior():
    do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_DISTANCE
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

# Ego: straight maneuver through intersection
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversary: straight maneuver conflicting with ego (from top/north)
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
    with color (1.0, 0.0, 0.0),  # Red ego vehicle
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (0.0, 0.0, 1.0),  # Blue adversarial vehicle
    with behavior AdvBehavior()

require monitor TrafficLights()

# Ensure adversary approaches from top (perpendicular crossing)
require CONST_MIN_PERP_DEG < (egoDir - advDir) < CONST_MAX_PERP_DEG

# Spawn distances from intersection center
require 30 <= (distance from egoSpawnPt to intersection) <= 45
require 15 <= (distance from advSpawnPt to intersection) <= 25