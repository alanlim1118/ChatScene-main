description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather ClearNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline

advSpawnPt = new OrientedPoint behind egoSpawnPt by Range(5, 10)

param EGO_REVERSE_SPEED = Range(2, 4)

behavior EgoBehavior():
    take SetReverseAction(True)
    do ConstantThrottleBehavior(0.3)

ego = new Car at egoSpawnPt,
    facing (egoSpawnPt.heading + 180 deg),
    with blueprint MODEL,
    with rolename 'hero',
    with behavior EgoBehavior()

behavior AdvBehavior():
    take SetHandBrakeAction(True)

adversary = new Car at advSpawnPt,
    facing egoSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdvBehavior()

param SPEED_LIMIT = 25

terminate when (distance from ego to adversary) < 1.5