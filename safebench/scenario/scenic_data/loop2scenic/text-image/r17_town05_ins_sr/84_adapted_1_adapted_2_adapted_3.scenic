description = "Using map ../../maps/Town05.xodr with carla map Town05 and weather ClearNoon"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(9, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero',
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

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