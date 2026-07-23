"""Scenario Description:

From a high-angle aerial perspective, a blue ego vehicle travels straight north through a four-way urban intersection surrounded by tall buildings under clear weather conditions. As the ego vehicle crosses the junction, an oncoming grey vehicle from the opposing northern arm executes a right turn onto the western cross street. Simultaneously, traffic enters from the western left arm: a red vehicle proceeds straight across the intersection towards the east, while a grey vehicle executes a left turn, heading south. The road features clear white lane markings, pedestrian crosswalks, and a bus stop zone marked on the pavement to the left of the ego vehicle's path.

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

param EGO_SPEED = Range(6, 10)
param ADV_ONCOMING_SPEED = Range(5, 9)
param ADV_WEST_STRAIGHT_SPEED = Range(5, 9)
param ADV_WEST_LEFT_SPEED = Range(5, 9)

EGO_COLOR = Color(0, 0, 1)          # Blue
GREY_COLOR = Color(0.5, 0.5, 0.5)   # Grey
RED_COLOR = Color(1, 0, 0)          # Red

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

behavior AdvBehavior(target_speed, trajectory):
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

# Ego: straight north from the southern arm (heading ~90 deg)
egoManeuver = Uniform(*filter(lambda m:
    m.type is ManeuverType.STRAIGHT and abs(m.startLane.centerline.end.heading - 90 deg) < 15 deg,
    intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

# Adv1: oncoming from north, right turn to west (heading ~-90 deg / 270 deg)
adv1Maneuver = Uniform(*filter(lambda m:
    m.type is ManeuverType.RIGHT_TURN and
    (abs(m.startLane.centerline.end.heading - (-90 deg)) < 15 deg or
     abs(m.startLane.centerline.end.heading - 270 deg) < 15 deg),
    intersection.maneuvers))
adv1InitLane = adv1Maneuver.startLane
adv1Trajectory = [adv1InitLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline

# Adv2: from west, straight to east (heading ~0 deg)
adv2Maneuver = Uniform(*filter(lambda m:
    m.type is ManeuverType.STRAIGHT and abs(m.startLane.centerline.end.heading - 0 deg) < 15 deg,
    intersection.maneuvers))
adv2InitLane = adv2Maneuver.startLane
adv2Trajectory = [adv2InitLane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]
adv2SpawnPt = new OrientedPoint in adv2InitLane.centerline

# Adv3: from west, left turn (heads north) (heading ~0 deg)
adv3Maneuver = Uniform(*filter(lambda m:
    m.type is ManeuverType.LEFT_TURN and abs(m.startLane.centerline.end.heading - 0 deg) < 15 deg,
    intersection.maneuvers))
adv3InitLane = adv3Maneuver.startLane
adv3Trajectory = [adv3InitLane, adv3Maneuver.connectingLane, adv3Maneuver.endLane]
adv3SpawnPt = new OrientedPoint in adv3InitLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color EGO_COLOR,
    with behavior EgoBehavior(egoTrajectory)

adversary1 = new Car at adv1SpawnPt,
    with blueprint ADV_MODEL,
    with color GREY_COLOR,
    with behavior AdvBehavior(globalParameters.ADV_ONCOMING_SPEED, adv1Trajectory)

adversary2 = new Car at adv2SpawnPt,
    with blueprint ADV_MODEL,
    with color RED_COLOR,
    with behavior AdvBehavior(globalParameters.ADV_WEST_STRAIGHT_SPEED, adv2Trajectory)

adversary3 = new Car at adv3SpawnPt,
    with blueprint ADV_MODEL,
    with color GREY_COLOR,
    with behavior AdvBehavior(globalParameters.ADV_WEST_LEFT_SPEED, adv3Trajectory)

# Ensure all vehicles are at reasonable distances from the intersection
require 20 <= (distance to intersection) <= 40
require 20 <= (distance from adversary1 to intersection) <= 40
require 20 <= (distance from adversary2 to intersection) <= 40
require 20 <= (distance from adversary3 to intersection) <= 40

terminate when (distance to egoSpawnPt) > 80