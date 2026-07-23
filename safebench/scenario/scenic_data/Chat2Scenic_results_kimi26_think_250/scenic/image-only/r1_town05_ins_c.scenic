description = "Pink ego vehicle cuts off a blue vehicle by merging diagonally into the left lane before a T-intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))
egoInitLane = Uniform(*filter(lambda l: len(l.group.lanes) == 2 and any(sec._laneToLeft is not None for sec in l.sections), intersection.incomingLanes))
egoLaneSec = Uniform(*filter(lambda sec: sec._laneToLeft is not None, egoInitLane.sections))
advLaneSec = egoLaneSec._laneToLeft
egoSpawnPt = new OrientedPoint on egoInitLane.centerline
advInitLane = advLaneSec.lane
advSpawnPt = new OrientedPoint on advInitLane.centerline
egoTrajectory = [egoInitLane]
advTrajectory = [advInitLane]

param OPT_EGO_SPEED = Range(6, 9)

behavior EgoBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectory)