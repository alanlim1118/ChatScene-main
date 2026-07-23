description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather ClearNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and m.intersection.is4Way and m.intersection.isSignalized, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero'

param OPT_ADV_SPEED = Range(12, 15)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param OPT_EGO_STOP_DIST = Range(5, 10)
param OPT_ADV_START_DIST = Range(25, 35)

require 30 <= (distance from egoSpawnPt to intersection) <= 40
require 20 <= (distance from advSpawnPt to intersection) <= 35
require 150 deg < (egoDir - advDir) < 210 deg

monitor TrafficSignalControl():
    freezeTrafficLights()
    setClosestTrafficLightStatus(ego, "red")
    setClosestTrafficLightStatus(adversary, "green")
    wait

require monitor TrafficSignalControl()

terminate when (distance from ego to adversary) < 0.5