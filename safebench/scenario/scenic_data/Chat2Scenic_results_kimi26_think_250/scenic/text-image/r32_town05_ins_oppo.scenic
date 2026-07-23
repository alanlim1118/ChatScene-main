description = "Ego vehicle waits behind a lead vehicle turning left at an urban intersection while an oncoming adversarial vehicle approaches."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEAD_DIST = Range(5, 10)

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoTrajectoryLine = egoManeuver.startLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

leadAdvSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST

oncomingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
oncomingLane = oncomingManeuver.startLane
oncomingAdvSpawnPt = new OrientedPoint in oncomingLane.centerline
oncomingTrajectory = [oncomingLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane]

param OPT_EGO_SPEED = Range(3, 5)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_LEAD_SPEED = Range(3, 5)

behavior LeadAdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_LEAD_SPEED, trajectory=egoTrajectory)

leadAdv = new Car at leadAdvSpawnPt,
    with heading leadAdvSpawnPt.heading,
    with behavior LeadAdvBehavior()

param OPT_ONCOMING_SPEED = Range(5, 10)

behavior OncomingAdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ONCOMING_SPEED, trajectory=oncomingTrajectory)

oncomingAdv = new Car at oncomingAdvSpawnPt,
    with heading oncomingAdvSpawnPt.heading,
    with color (0, 255, 0),
    with behavior OncomingAdvBehavior()