description = "Ego vehicle travels straight, then avoids a stationary bicycle directly in its path."
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

bikeSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(20, 30)

param EGO_SPEED = Range(7, 10)
param SAFETY_DISTANCE = 15

behavior EgoBehavior(speed, safety_dist):
	try:
		do FollowLaneBehavior(target_speed=speed)
	interrupt when withinDistanceToObjsInLane(self, safety_dist):
		take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.SAFETY_DISTANCE)

bicycle = new Bicycle at bikeSpawnPt

require 20 <= (distance from egoSpawnPt to bikeSpawnPt) <= 30
terminate when (distance to bicycle) <= 15 or (distance to egoSpawnPt) > 50