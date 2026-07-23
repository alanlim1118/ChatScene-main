description = "Vehicle pulls out from a parking space adjacent to a handicapped spot and encounters a dog standing in its path."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

param OPT_DOG_DISTANCE = Range(5, 15)

egoLane = Uniform(*network.lanes)
lanePt = new OrientedPoint in egoLane.centerline
egoSpawnPt = new OrientedPoint right of lanePt by Range(2, 3), facing roadDirection
dogSpawnPt = new OrientedPoint ahead of lanePt by globalParameters.OPT_DOG_DISTANCE

param OPT_EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED, laneToFollow=egoLane)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

dog = new Pedestrian at dogSpawnPt