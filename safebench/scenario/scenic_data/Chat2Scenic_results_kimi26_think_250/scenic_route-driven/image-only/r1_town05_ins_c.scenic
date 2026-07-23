description = "Pink ego vehicle cuts off a blue vehicle by merging diagonally into the left lane before a T-intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
advLaneSec = egoLaneSec._laneToLeft
advInitLane = advLaneSec.lane
advSpawnPt = new OrientedPoint on advInitLane.centerline
egoTrajectory = [egoInitLane]
advTrajectory = [advInitLane]

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectory)