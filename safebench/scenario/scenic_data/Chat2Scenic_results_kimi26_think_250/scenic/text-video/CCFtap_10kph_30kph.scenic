description = "Ego vehicle traveling straight through an intersection is struck by a left-turning red vehicle that fails to yield."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

behavior EgoBehavior():
    do FollowTrajectoryBehavior(target_speed=10, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=10, trajectory=advTrajectory)

adv = new Car at advSpawnPt,
    with behavior AdvBehavior()

require 20 <= (distance from egoSpawnPt to intersection) <= 50
require 10 <= (distance from advSpawnPt to intersection) <= 40
terminate when ((ego intersects adv) and (intersection intersects ego) and (intersection intersects adv))