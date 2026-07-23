description = "Ego vehicle travels on a dark highway at night when a pedestrian emerges from the left, illuminated only by headlights, forcing emergency braking and swerving."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way and not m.intersection.isSignalized, egoInitLane.maneuvers))

IntSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(20, 35)
pedSpawnPt = new OrientedPoint left of IntSpawnPt by Range(3, 6)

ego = new Car at egoSpawnPt,
    with blueprint MODEL

behavior PedestrianBehavior():
    do CrossingBehavior(ego, 1, 25)

ped = new Pedestrian at pedSpawnPt,
    facing toward IntSpawnPt,
    with regionContainedIn None,
    with behavior PedestrianBehavior()