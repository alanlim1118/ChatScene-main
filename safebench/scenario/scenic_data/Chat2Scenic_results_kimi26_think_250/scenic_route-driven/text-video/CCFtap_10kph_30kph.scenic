description = "Ego vehicle traveling straight through an intersection is struck by a left-turning red vehicle that fails to yield."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
    with blueprint MODEL

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=10, trajectory=advTrajectory)

adv = new Car at advSpawnPt,
    with behavior AdvBehavior()

require 20 <= (distance from egoSpawnPt to intersection) <= 50
require 10 <= (distance from advSpawnPt to intersection) <= 40
terminate when ((ego intersects adv) and (intersection intersects ego) and (intersection intersects adv))
