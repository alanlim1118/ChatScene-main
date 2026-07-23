description = "Ego vehicle travels straight in the center lane while an adversarial vehicle cuts across from the right lane to the left lane in front of it."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

rightLaneSec = egoLaneSec._laneToRight
leftLaneSec = egoLaneSec._laneToLeft

advRefPt = rightLaneSec.centerline.project(egoSpawnPt.position)
param OPT_ADV_START_DIST = Range(-10, -5)
advSpawnPt = new OrientedPoint following egoLaneSec.orientation from advRefPt for globalParameters.OPT_ADV_START_DIST

rightLane = rightLaneSec.lane
centerLane = egoLaneSec.lane
leftLane = leftLaneSec.lane
advTrajectory = [rightLane, centerLane, leftLane]

param EGO_SPEED = Range(7, 10)

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_SPEED = globalParameters.EGO_SPEED + Range(2, 5)

behavior AdvBehavior(target_speed):
    do FollowTrajectoryBehavior(target_speed, advTrajectory)
    terminate

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)