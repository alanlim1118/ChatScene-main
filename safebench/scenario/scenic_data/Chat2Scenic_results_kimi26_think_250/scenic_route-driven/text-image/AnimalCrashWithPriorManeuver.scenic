description = "Vehicle pulls out from a parking space adjacent to a handicapped spot and encounters a dog standing in its path."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

param OPT_DOG_DISTANCE = Range(5, 15)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
lanePt = new OrientedPoint left of egoSpawnPt by Range(2, 3), facing egoSpawnPt.heading
dogSpawnPt = new OrientedPoint ahead of lanePt by globalParameters.OPT_DOG_DISTANCE

ego = new Car at egoSpawnPt,
    with blueprint MODEL

dog = new Pedestrian at dogSpawnPt