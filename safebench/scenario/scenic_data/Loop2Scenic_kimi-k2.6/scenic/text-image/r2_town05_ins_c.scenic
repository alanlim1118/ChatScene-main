description = "Ego vehicle follows a lead vehicle in a rural area, which suddenly decelerates."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_FOLLOW_DISTANCE = Range(10, 20)

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
leadSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_FOLLOW_DISTANCE

param EGO_SPEED = Range(7, 10)
param SAFETY_DISTANCE = 15

behavior EgoBehavior(speed, safety_dist):
	try:
		do FollowLaneBehavior(target_speed=speed)
	interrupt when withinDistanceToObjsInLane(self, safety_dist):
		take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.SAFETY_DISTANCE)

param ADV_SPEED = Range(7, 10)
param ADV_BRAKE = 1.0
param ADV_DURATION = Range(2, 5)

behavior AdvBehavior(speed, brake_val, duration):
	do FollowLaneBehavior(target_speed=speed) for duration seconds
	while True:
		take SetBrakeAction(brake_val)

lead = new Car at leadSpawnPt,
	with blueprint MODEL,
	with behavior AdvBehavior(globalParameters.ADV_SPEED, globalParameters.ADV_BRAKE, globalParameters.ADV_DURATION)

require 10 <= (distance from egoSpawnPt to leadSpawnPt) <= 20