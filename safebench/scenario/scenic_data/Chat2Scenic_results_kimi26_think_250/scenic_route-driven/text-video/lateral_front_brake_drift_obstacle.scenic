description = "Ego vehicle follows a lead car that brakes suddenly, forcing an emergency right swerve into a stationary blue advertising sign."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEAD_DIST = Range(10, 20)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

LeadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST

rightLaneSec = egoLaneSec._laneToRight
signLanePt = rightLaneSec.centerline.project(LeadSpawnPt.position)
SignSpawnPt = new OrientedPoint at signLanePt

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = Range(5, 8)
param OPT_ADV_BRAKE_TIME = Range(2, 4)

behavior AdvBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) for globalParameters.OPT_ADV_BRAKE_TIME seconds
	while True:
		take SetBrakeAction(1)

AdvAgent = new Car at LeadSpawnPt,
	with heading LeadSpawnPt.heading,
	with regionContainedIn egoLaneSec,
	with behavior AdvBehavior()

advertisement = new Advertisement at SignSpawnPt

require 10 <= (distance from ego to AdvAgent) <= 20
terminate when ego intersects advertisement