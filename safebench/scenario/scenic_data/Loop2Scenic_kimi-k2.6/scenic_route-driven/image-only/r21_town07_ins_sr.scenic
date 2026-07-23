description = "The ego vehicle is entering the intersection; the adversarial vehicle comes from the opposite direction and turns left and stops, causing a near collision with the ego vehicle."

Town = 'Town05'
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *
EGO_MODEL = "vehicle.lincoln.mkz_2017"

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.intersection is not None and m.intersection.is4Way and m.intersection.isSignalized, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))

advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

egoTrajectoryLine = egoManeuver.startLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn None,
	with blueprint EGO_MODEL

param OPT_ADV_DISTANCE = Range(10, 20)/10
param OPT_ADV_SPEED = Range (1, 5) * Range(10, 15)/10

behavior AdvBehavior():
	do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to egoTrajectoryLine <= globalParameters.OPT_ADV_DISTANCE)
	take SetThrottleAction(0)
	take SetBrakeAction(1)

AdvAgent = new Car at advSpawnPt,
	with heading advSpawnPt.heading,
	with regionContainedIn None,
	with behavior AdvBehavior()

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

CONST_OPPOSITE_DEG = 180 deg
CONST_TOL_DEG = 20 deg
CONST_MIN_OPPOSITE_DEG = CONST_OPPOSITE_DEG - CONST_TOL_DEG
CONST_MAX_OPPOSITE_DEG = CONST_OPPOSITE_DEG + CONST_TOL_DEG

require CONST_MIN_OPPOSITE_DEG < (egoDir - advDir) < CONST_MAX_OPPOSITE_DEG
require 30 <= (distance from egoSpawnPt to intersection) <= 40
require 10 <= (distance from advSpawnPt to intersection) <= 20

MODEL = 'vehicle.lincoln.mkz_2017'

param weather = 'SoftRainNoon'
