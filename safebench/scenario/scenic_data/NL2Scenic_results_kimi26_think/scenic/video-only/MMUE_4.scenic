"""Scenario Description:

In a top-down view of a T-junction, the ego vehicle is positioned on the vertical minor road attempting to turn right onto the horizontal main road. Its maneuver is impeded by a queue of yellow adversary vehicles approaching from the right arm of the junction. These adversaries are traveling along the main road; while some proceed straight through the intersection, others are executing left turns into the minor road, directly crossing the ego vehicle's path. Consequently, the ego vehicle is forced to yield and wait for the stream of cross-traffic and turning vehicles to clear the intersection before it can safely proceed.

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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_ADV_SPEED = Range(5, 10)
param OPT_YIELD_DIST = Range(12, 20)
param OPT_EGO_BRAKE = Range(0.5, 1.0)

TERM_DIST = 70

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_YIELD_DIST):
        take SetBrakeAction(globalParameters.OPT_EGO_BRAKE)

behavior AdvBehavior(trajectory, speed):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

# Ego: right turn from the minor road onto the main road
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Right arm: incoming lane with left-turning traffic that crosses ego's path
advInitLane = Uniform(*filter(lambda l:
    any(m.type is ManeuverType.LEFT_TURN and m.startLane is l for m in egoManeuver.conflictingManeuvers),
    set(m.startLane for m in egoManeuver.conflictingManeuvers)))

advStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.startLane is advInitLane, egoManeuver.conflictingManeuvers))
advLeftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.startLane is advInitLane, egoManeuver.conflictingManeuvers))

advStraightTrajectory = [advStraightManeuver.startLane, advStraightManeuver.connectingLane, advStraightManeuver.endLane]
advLeftTrajectory = [advLeftManeuver.startLane, advLeftManeuver.connectingLane, advLeftManeuver.endLane]

# Queue of adversaries approaching from the right arm
advSpawnPt1 = new OrientedPoint in advInitLane.centerline
advSpawnPt2 = new OrientedPoint in advInitLane.centerline
advSpawnPt3 = new OrientedPoint in advInitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(egoTrajectory)

# Closest adversary executes a left turn, crossing directly in front of ego
adv1 = new Car at advSpawnPt1,
    with heading advSpawnPt1.heading,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior(advLeftTrajectory, globalParameters.OPT_ADV_SPEED)

# Second adversary proceeds straight through the intersection
adv2 = new Car at advSpawnPt2,
    with heading advSpawnPt2.heading,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior(advStraightTrajectory, globalParameters.OPT_ADV_SPEED)

# Third adversary proceeds straight through the intersection
adv3 = new Car at advSpawnPt3,
    with heading advSpawnPt3.heading,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior(advStraightTrajectory, globalParameters.OPT_ADV_SPEED)

# Spatial constraints to form an approaching queue on the right arm
require 15 <= (distance to intersection) <= 25
require 5 <= (distance from advSpawnPt1 to intersection) <= 10
require 20 <= (distance from advSpawnPt2 to intersection) <= 30
require 35 <= (distance from advSpawnPt3 to intersection) <= 45

terminate when (distance to egoSpawnPt) > TERM_DIST