description = "Ego vehicle on a rural straight road encounters an animal standing in its path."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for Range(30, 80)

ego = new Car at egoSpawnPt,
    with blueprint MODEL

behavior StationaryAnimalBehavior():
    while True:
        wait

animal = new Pedestrian at advSpawnPt,
    with regionContainedIn None,
    with behavior StationaryAnimalBehavior()

terminate when (distance from egoSpawnPt to ego) > 100