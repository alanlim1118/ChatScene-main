description = "Ego vehicle executes a right turn at an intersection while an adversarial object entering from the left lane makes a left turn."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.intersection is not None and m.intersection.is4Way and m.startLane.sections and m.startLane.sections[-1]._laneToLeft is not None and any(n.type is ManeuverType.LEFT_TURN for n in m.startLane.sections[-1]._laneToLeft.lane.maneuvers), egoInitLane.maneuvers))
egoLaneSec = egoInitLane.sections[-1]
advInitLane = egoLaneSec._laneToLeft.lane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))
advSpawnPt = new OrientedPoint on advInitLane.centerline
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param ADV_SPEED = 10

behavior AdvBehavior(trajectory, target_speed):
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)

adv = new Car at advSpawnPt,
    with regionContainedIn None,
    with behavior AdvBehavior(advTrajectory, globalParameters.ADV_SPEED)