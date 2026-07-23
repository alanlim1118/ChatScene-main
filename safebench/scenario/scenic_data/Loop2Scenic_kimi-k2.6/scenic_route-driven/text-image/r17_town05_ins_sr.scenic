description = "Using map ../../maps/Town05.xodr with carla map Town05 and weather ClearNoon"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuverCands = [m for m in egoInitLane.maneuvers if m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way]
intersection = egoManeuverCands[0].intersection
egoManeuver = Uniform(*[m for m in egoManeuverCands if m.intersection is intersection])

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero'

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

buildingSpawnPt = new OrientedPoint at egoSpawnPt offset by (10, 10, 0)
building = new Prop at buildingSpawnPt,
    with blueprint 'static.prop.container'

buildingSpawnPt = new OrientedPoint at egoSpawnPt offset by (10, -10, 0)
building = new Prop at buildingSpawnPt,
    with blueprint 'static.prop.container'

treeSpawnPt = new OrientedPoint at egoSpawnPt offset by (Range(-15, 15), Range(-15, 15), 0)
tree = new Prop at treeSpawnPt,
    with blueprint 'static.prop.plantpot01'

require 10 <= (distance from egoSpawnPt to intersection) <= 60
require 10 <= (distance from advSpawnPt to intersection) <= 60
terminate when (distance from ego to intersection > 80 or distance from adversary to intersection > 80)