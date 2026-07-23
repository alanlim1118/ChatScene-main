"""Scenario Description:

The ego vehicle is turning left at an intersection; the adversarial pedestrian on the right of the target lane suddenly crosses the road and stops in the middle of the road.

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

param OPT_ADV_SPEED = Range(1, 5)
param OPT_ADV_DISTANCE = Range(15, 20)

OPT_STOP_DISTANCE = 1
OPT_PARAM_LANE_WIDTH = 6

#################################
# AGENT BEHAVIORS               #
#################################

behavior CrossAndStopBehavior(actor_reference, adv_speed, adv_distance, stop_reference, stop_distance):
    do CrossingBehavior(actor_reference, adv_speed, adv_distance) until (distance from self to stop_reference <= stop_distance)
    take SetWalkingSpeedAction(0)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
intersection = egoManeuver.intersection
egoTrajectoryLine = egoManeuver.startLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline

# Spawn point on the far side of the intersection, along the end lane's centerline
endLanePt = new OrientedPoint at egoManeuver.endLane.rightEdge.start,
    with heading egoInitLane.centerline.end.heading - 180 deg
pedSpawnPt = new OrientedPoint ahead of endLanePt by - OPT_PARAM_LANE_WIDTH

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL

AdvAgent = new Pedestrian at pedSpawnPt,
    with heading pedSpawnPt.heading,  # Perpendicular to the road, crossing the street
    with regionContainedIn None,
    with behavior CrossAndStopBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE, egoTrajectoryLine, OPT_STOP_DISTANCE)

require 40 <= (distance to intersection) <= 60
