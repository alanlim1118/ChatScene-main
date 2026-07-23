description = "Ego vehicle goes straight in an urban non-junction area and takes evasive action to avoid an obstacle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
propSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(15, 40)

param EGO_SPEED = Range(5, 10)
param EGO_BRAKE = Range(0.5, 1.0)
SAFE_DIST = 15

behavior EgoBehavior():
	try:
		do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
	interrupt when withinDistanceToAnyObjs(self, SAFE_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

adversary = new Debris at propSpawnPt