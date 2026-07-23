description = "A sedan swerves left across the gore area and cuts across the ego vehicle's path, causing a side-impact collision near an overpass."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
advLaneSec = egoLaneSec._laneToRight
egoInitLane = egoLaneSec.lane
advInitLane = advLaneSec.lane
advSpawnPt = new OrientedPoint in advLaneSec.centerline
egoTrajectory = [egoInitLane]
advTrajectory = [advInitLane]

ego = new Car at egoSpawnPt,
    with blueprint MODEL

behavior AdvBehavior():
    do LaneChangeBehavior(egoLaneSec, target_speed=15)
    do FollowLaneBehavior(target_speed=15)

adv = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with behavior AdvBehavior()