description = "Ego vehicle follows a lead vehicle in a rural area, which suddenly decelerates."
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_FOLLOW_DISTANCE = Range(10, 20)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
leadSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_FOLLOW_DISTANCE

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)
param ADV_BRAKE = 1.0
param OPT_ADV_DURATION = Range(2, 5)

behavior AdvBehavior(speed, brake_val, duration):
	do FollowLaneBehavior(target_speed=speed) for duration seconds
	while True:
		take SetBrakeAction(brake_val)

lead = new Car at leadSpawnPt,
	with blueprint MODEL,
	with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.ADV_BRAKE, globalParameters.OPT_ADV_DURATION)

require 10 <= (distance from egoSpawnPt to leadSpawnPt) <= 20