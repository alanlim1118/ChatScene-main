description = "Ego vehicle traveling forward approaches a lead vehicle that is exiting the flow of traffic by turning or reversing toward the left."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
advSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(15, 40)

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(3, 5)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for Range(2, 4) seconds
	take SetSteerAction(steer=1.0)
	take SetThrottleAction(throttle=0.5)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()