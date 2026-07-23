description = "Ego vehicle travels straight through a rural four-way intersection as two adversary vehicles turn right from the right arm."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(6, 10)
param EGO_BRAKE = Range(0.5, 1.0)
SAFE_DIST = 20

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, SAFE_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with color red,
	with behavior AdversaryBehavior(advTrajectory)

param GREY_ADV_SPEED = Range(7, 10)

behavior GreyCarBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.GREY_ADV_SPEED, trajectory=trajectory)

greyCar = new Car behind advSpawnPt by 10,
	with color grey,
	with behavior GreyCarBehavior(advTrajectory)