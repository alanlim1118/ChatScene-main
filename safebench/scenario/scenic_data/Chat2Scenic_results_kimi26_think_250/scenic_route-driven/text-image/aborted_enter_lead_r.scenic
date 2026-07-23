description = "Ego vehicle in center lane with adversary ahead weaving from right lane toward center and back."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_DIST = Range(15, 25)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
rightLaneSec = egoLaneSec._laneToRight

adjLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_DIST

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(2, 4)

behavior AdversaryBehavior():
	centerLaneSec = self.laneSection._laneToLeft
	do LaneChangeBehavior(
		laneSectionToSwitch=centerLaneSec,
		target_speed=globalParameters.OPT_ADV_SPEED)
	rightLaneSec = self.laneSection._laneToRight
	do LaneChangeBehavior(
		laneSectionToSwitch=rightLaneSec,
		target_speed=globalParameters.OPT_ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at AdvSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()