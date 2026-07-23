description = "Ego vehicle travels straight within its lane following an adversarial object ahead overlapping from the left."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
intSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(20, 40)
advSpawnPt = new OrientedPoint left of intSpawnPt by Range(1.5, 2.0)

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

require 20 <= (distance from egoSpawnPt to advSpawnPt) <= 40