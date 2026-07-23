description = "A sedan swerves left across the gore area and cuts across the ego vehicle's path, causing a side-impact collision near an overpass."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

advLaneSec = Uniform(*filter(lambda s: s._laneToRight is None and s._laneToLeft is not None, [sec for lane in network.lanes for sec in lane.sections]))
egoLaneSec = advLaneSec._laneToLeft
egoInitLane = egoLaneSec.lane
advInitLane = advLaneSec.lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in advLaneSec.centerline
egoTrajectory = [egoInitLane]
advTrajectory = [advInitLane]

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=10)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

behavior AdvBehavior():
    do LaneChangeBehavior(egoLaneSec, target_speed=15)
    do FollowLaneBehavior(target_speed=15)

adv = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with behavior AdvBehavior()