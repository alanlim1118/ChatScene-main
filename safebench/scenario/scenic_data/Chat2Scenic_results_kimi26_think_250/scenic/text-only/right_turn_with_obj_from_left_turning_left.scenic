description = "Ego vehicle executes a right turn at an intersection while an adversarial object entering from the left lane makes a left turn."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.startLane.sections and m.startLane.sections[-1]._laneToLeft is not None and any(n.type is ManeuverType.LEFT_TURN for n in m.startLane.sections[-1]._laneToLeft.lane.maneuvers), intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoLaneSec = egoInitLane.sections[-1]
advInitLane = egoLaneSec._laneToLeft.lane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, advInitLane.maneuvers))
egoSpawnPt = new OrientedPoint on egoInitLane.centerline
advSpawnPt = new OrientedPoint on advInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = 10

behavior EgoBehavior(trajectory, target_speed):
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory, globalParameters.EGO_SPEED)

param ADV_SPEED = 10

behavior AdvBehavior(trajectory, target_speed):
    do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=trajectory)

adv = new Car at advSpawnPt,
    with regionContainedIn None,
    with behavior AdvBehavior(advTrajectory, globalParameters.ADV_SPEED)