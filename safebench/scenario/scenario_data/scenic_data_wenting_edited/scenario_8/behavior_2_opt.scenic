description = "Ego vehicle approaches intersection; adversarial car from right suddenly accelerates, enters first, and stops."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(trajectory=trajectory, target_speed=10)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param OPT_ADV_SPEED = Range(2, 5)
param OPT_ADV_ACCEL_SPEED = Range(12, 18)
param OPT_ADV_DISTANCE = Range(2, 5)
param OPT_ADV_TIMER = Range(1.5, 2.5)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory) until (distance from self to intersection <= globalParameters.OPT_ADV_DISTANCE)
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_ACCEL_SPEED, trajectory=advTrajectory) for globalParameters.OPT_ADV_TIMER seconds
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    terminate

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

require 30 <= (distance from egoSpawnPt to intersection) <= 40