description = "Vehicle backs up in an urban driveway or alley and collides with another vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
advSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(-10, -5)

param EGO_REVERSE_THROTTLE = Range(0.3, 0.5)

behavior EgoBehavior():
	take SetReverseAction(True)
	while True:
		take SetThrottleAction(globalParameters.EGO_REVERSE_THROTTLE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

stationary_adv = new Car at advSpawnPt,
	with blueprint MODEL