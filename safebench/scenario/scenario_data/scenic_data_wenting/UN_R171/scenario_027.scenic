description = "Ego vehicle travels straight, encounters stationary pedestrian in path, requiring autonomous braking to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DISTANCE = Range(25, 35)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

pedestrianSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

behavior StationaryBehavior():
    while True:
        wait

pedestrian = new Pedestrian at pedestrianSpawnPt,
    with regionContainedIn None,
    with behavior StationaryBehavior()

TERM_DIST = 70

require 25 <= (distance from egoSpawnPt to pedestrianSpawnPt) <= 35
terminate when (distance to egoSpawnPt) > TERM_DIST
