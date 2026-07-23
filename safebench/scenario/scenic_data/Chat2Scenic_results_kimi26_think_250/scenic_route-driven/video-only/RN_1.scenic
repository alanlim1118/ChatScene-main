description = "Ego vehicle navigates a roundabout in the outer lane alongside a red adversary to execute a leftward turn."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

advInitLane = egoInitLane.sectionAt(egoSpawnPt).laneToLeft.lane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoManeuver = Uniform(*filter(lambda m: m.intersection is not None, egoInitLane.maneuvers))
intersection = egoManeuver.intersection
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*advInitLane.maneuvers)
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(5, 8)

behavior AdversaryBehavior():
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory)

adversary = new Car at advSpawnPt,
    with color Color(1, 0, 0),
    with behavior AdversaryBehavior()

require 15 <= (distance from egoSpawnPt to intersection) <= 50
require 15 <= (distance from advSpawnPt to intersection) <= 50
terminate when (ego in egoManeuver.endLane) and ((distance from ego to egoSpawnPt) > 15)