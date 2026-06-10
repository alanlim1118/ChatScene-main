description = "Ego vehicle approaches a slower moving heavy truck or motorcycle from behind on a straight road."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advInitLane = egoInitLane
advSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(20, 40)

egoTrajectory = [egoInitLane]
advTrajectory = [advInitLane]

param EGO_SPEED = Range(15, 20)

behavior EgoBehavior(speed, trajectory):
	do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.EGO_SPEED, egoTrajectory)

param OPT_ADV_SPEED = globalParameters.EGO_SPEED - Range(5, 10)

behavior AdversarialBehavior(speed, trajectory):
	do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

advVehicle = new Truck at advSpawnPt,
	with behavior AdversarialBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory)

param TERMINATE_DISTANCE = 50
param TERMINATE_TIME = 60

require 20 <= (distance from egoSpawnPt to advSpawnPt) <= 40
require -10 deg < (relative heading of egoSpawnPt from advSpawnPt) < 10 deg

terminate when (distance from ego to advVehicle) > globalParameters.TERMINATE_DISTANCE
terminate after globalParameters.TERMINATE_TIME seconds