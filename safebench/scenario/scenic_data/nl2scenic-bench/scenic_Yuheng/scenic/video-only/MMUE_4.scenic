"""Scenario Description:

In a top-down view of a T-junction, the ego vehicle, represented by a green box, is positioned on the vertical minor road attempting to turn right onto the horizontal main road. Its maneuver is impeded by a queue of yellow adversary vehicles approaching from the right arm of the junction. These adversaries are traveling along the main road; while some proceed straight through the intersection, others are executing left turns into the minor road, directly crossing the ego vehicle's path. Consequently, the ego vehicle is forced to yield and wait for the stream of cross-traffic and turning vehicles to clear the intersection before it can safely proceed.

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

param EGO_SPEED = Range(3, 6)
param ADV_SPEED = Range(8, 12)
param SAFETY_DIST = Range(12, 18)
param BRAKE_INTENSITY = Range(0.7, 1.0)
param QUEUE_SPACING = Range(25, 40)
TERM_DIST = 80

#################################
# MONITORS                      #
#################################

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        for adv in adversaries:
            if withinDistanceToTrafficLight(adv, 100):
                setClosestTrafficLightStatus(adv, "green")
        wait

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(globalParameters.BRAKE_INTENSITY)
    interrupt when withinDistanceToAnyObjs(self, 3):
        terminate

behavior AdvBehavior(trajectory, delay):
    wait for delay seconds
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a 3-way (T) intersection
intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego is on the minor road making a RIGHT TURN onto the main road
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adversaries approach from the RIGHT arm of the junction relative to ego
# The right arm corresponds to lanes whose straight maneuvers conflict with ego's right turn
rightArmLanes = Uniform(*filter(
    lambda m: m.type is ManeuverType.STRAIGHT and m in egoManeuver.conflictingManeuvers,
    intersection.maneuvers
)).startLane

# Each adversary either goes STRAIGHT or makes a LEFT TURN into the minor road
def getAdvManeuver(lane):
    return Uniform(
        *filter(lambda m: m.type in (ManeuverType.STRAIGHT, ManeuverType.LEFT_TURN), lane.maneuvers)
    )

advManeuver1 = getAdvManeuver(rightArmLanes)
advTrajectory1 = [rightArmLanes, advManeuver1.connectingLane, advManeuver1.endLane]
advSpawnPt1 = new OrientedPoint in rightArmLanes.centerline

# Second adversary further back in the queue
advManeuver2 = getAdvManeuver(rightArmLanes)
advTrajectory2 = [rightArmLanes, advManeuver2.connectingLane, advManeuver2.endLane]
advSpawnPt2 = new OrientedPoint in rightArmLanes.centerline

# Third adversary even further back
advManeuver3 = getAdvManeuver(rightArmLanes)
advTrajectory3 = [rightArmLanes, advManeuver3.connectingLane, advManeuver3.endLane]
advSpawnPt3 = new OrientedPoint in rightArmLanes.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color (0, 1, 0),
    with behavior EgoBehavior(egoTrajectory)

adversaries = []

adv1 = new Car at advSpawnPt1,
    with blueprint ADV_MODEL,
    with color (1, 1, 0),
    with behavior AdvBehavior(advTrajectory1, 0)
adversaries.append(adv1)

adv2 = new Car at advSpawnPt2,
    with blueprint ADV_MODEL,
    with color (1, 1, 0),
    with behavior AdvBehavior(advTrajectory2, Range(1, 3))
adversaries.append(adv2)

adv3 = new Car at advSpawnPt3,
    with blueprint ADV_MODEL,
    with color (1, 1, 0),
    with behavior AdvBehavior(advTrajectory3, Range(3, 5))
adversaries.append(adv3)

require monitor TrafficLights()

# Ego starts at a reasonable distance from the intersection on the minor road
require 20 <= (distance from egoSpawnPt to intersection) <= 35

# Adversaries are queued along the right arm approaching the intersection
require 15 <= (distance from advSpawnPt1 to intersection) <= 30
require (distance from advSpawnPt1 to advSpawnPt2) >= globalParameters.QUEUE_SPACING[0]
require (distance from advSpawnPt1 to advSpawnPt2) <= globalParameters.QUEUE_SPACING[1]
require (distance from advSpawnPt2 to advSpawnPt3) >= globalParameters.QUEUE_SPACING[0]
require (distance from advSpawnPt2 to advSpawnPt3) <= globalParameters.QUEUE_SPACING[1]

# Ensure all spawn points are in distinct positions (no overlap)
require (distance from advSpawnPt1 to advSpawnPt2) > 10
require (distance from advSpawnPt2 to advSpawnPt3) > 10

terminate when (distance from ego to egoSpawnPt) > TERM_DIST