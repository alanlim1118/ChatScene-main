description = "Ego vehicle turns right at an intersection and strikes a crossing pedestrian without braking."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
pedInitLane = egoManeuver.endLane
pedSpawnPt = new OrientedPoint in pedInitLane.centerline

behavior EgoBehavior():
    do FollowTrajectoryBehavior(target_speed=10, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

behavior PedCrossingBehavior():
    do CrossingBehavior(ego)

adversary = new Pedestrian at pedSpawnPt,
    facing 90 deg relative to pedSpawnPt.heading,
    with behavior PedCrossingBehavior()

require 5 <= (distance from egoSpawnPt to intersection) <= 25
require 5 <= (distance from egoSpawnPt to pedSpawnPt) <= 40
terminate when (ego in pedInitLane)