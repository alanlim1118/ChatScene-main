description = "Two vehicles enter a four-way intersection from parallel bottom lanes; the left vehicle proceeds straight while the right vehicle executes a U-turn."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: any(s._laneToRight is not None for s in m.startLane.sections), filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers)))
egoInitLane = egoManeuver.startLane
egoLaneSec = Uniform(*filter(lambda s: s._laneToRight is not None, egoInitLane.sections))
advLaneSec = egoLaneSec._laneToRight
advInitLane = advLaneSec.lane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.UTURN, advInitLane.maneuvers))
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint

param EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(5, 10)

behavior UTurnBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=[advInitLane, advManeuver.connectingLane])

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with blueprint MODEL,
    with regionContainedIn advLaneSec,
    with behavior UTurnBehavior()