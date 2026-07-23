description = "Ego vehicle travels straight through a rural intersection while an adversary vehicle cuts over from the adjacent right lane and merges in front of the ego."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoInitLane = Uniform(*filter(lambda lane: any(sec._laneToRight is not None for sec in lane.sections), intersection.incomingLanes))
egoLaneSec = Uniform(*filter(lambda sec: sec._laneToRight is not None, egoInitLane.sections))
advLaneSec = egoLaneSec._laneToRight
advInitLane = advLaneSec.lane
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline

param OPT_EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED + Range(1, 3)
param OPT_ADV_MERGE_DIST = Range(15, 25)

behavior AdvBehavior(target_speed, merge_distance):
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego < merge_distance)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn advLaneSec,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_MERGE_DIST)