"""Scenario Description:

In a rural environment characterized by a T-junction, a main vertical road intersects with a side road entering from the left, flanked by a field with scattered hay bales and trees on the west and a body of water on the east. A purple vehicle, serving as the ego car, travels straight along the southern lane of the main road towards the intersection. At the junction, a blue adversarial vehicle is turning right from the side road, merging into the lane directly ahead of the ego car, with its path indicated by a cyan trajectory line. Further north in the same lane, a red vehicle drives straight away from the intersection, leaving a pink trajectory trace behind it, presenting a scenario where the ego car must account for the vehicle cutting in from the side.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'  # Rural map with T-junctions and natural scenery
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"
LEAD_MODEL = "vehicle.audi.a2"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(0.9, 1.0, 1.1)
param OPT_LEAD_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.0, 1.1, 1.2)
param OPT_ADV_DISTANCE = Range(50, 70)  # Distance at which adversarial begins maneuver
param OPT_BRAKE_DISTANCE = Range(8, 12)  # Ego braking threshold

CONST_RIGHT_DEG = -90 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_RIGHT_DEG = CONST_RIGHT_DEG - CONST_TOL_DEG
CONST_MAX_RIGHT_DEG = CONST_RIGHT_DEG + CONST_TOL_DEG

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
        if withinDistanceToTrafficLight(LeadVehicle, 100):
            setClosestTrafficLightStatus(LeadVehicle, "green")
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
        take SetThrottleAction(0)
        take SetBrakeAction(1)

behavior AdvBehavior():
    do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_DISTANCE
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)
    terminate

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a T-junction (3-way intersection)
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego goes straight through the T-junction from the south
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversarial turns right from the left side road into ego's lane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoManeuver.conflictingManeuvers))
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

# Lead vehicle is further north in the same lane as ego's end lane, driving straight away
leadLane = egoManeuver.endLane
leadSpawnPt = new OrientedPoint in leadLane.centerline

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with color (0.5, 0.0, 0.5),  # Purple
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (0.0, 0.0, 0.8),  # Blue
    with behavior AdvBehavior()

LeadVehicle = new Car at leadSpawnPt,
    with heading leadLane.centerline.heading,
    with regionContainedIn None,
    with blueprint LEAD_MODEL,
    with color (0.8, 0.0, 0.0),  # Red
    with behavior LeadBehavior()

require monitor TrafficLights()
require CONST_MIN_RIGHT_DEG < (advDir - egoDir) < CONST_MAX_RIGHT_DEG
require 30 <= (distance from egoSpawnPt to intersection) <= 50
require 10 <= (distance from advSpawnPt to intersection) <= 25
require 40 <= (distance from leadSpawnPt to intersection) <= 60
require (distance from egoSpawnPt to leadSpawnPt) > 60