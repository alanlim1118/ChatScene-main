description = "Ego vehicle executes a free right turn at a four-way intersection, transitioning from the southern approach onto the eastern cross street."
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

ego = new Car at egoSpawnPt,
    with blueprint MODEL