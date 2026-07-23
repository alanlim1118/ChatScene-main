description = "Ego vehicle turns right at an intersection and strikes a crossing pedestrian without braking."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection
pedInitLane = egoManeuver.endLane
pedSpawnPt = new OrientedPoint in pedInitLane.centerline

ego = new Car at egoSpawnPt,
    with blueprint MODEL

behavior PedCrossingBehavior():
    do CrossingBehavior(ego)

adversary = new Pedestrian at pedSpawnPt,
    facing 90 deg relative to pedSpawnPt.heading,
    with behavior PedCrossingBehavior()

require 5 <= (distance from egoSpawnPt to intersection) <= 25
require 5 <= (distance from egoSpawnPt to pedSpawnPt) <= 40
terminate when (ego in pedInitLane)
