description = "Ego vehicle detects and reacts to a stationary target vehicle in a curve, positioned with a 0.5m lane offset."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param ADV_OFFSET = 0.5
param OPT_DIST_TO_CURVE = Range(40, 60)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLane = network.laneAt(egoSpawnPt.position)

curvePoint = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.OPT_DIST_TO_CURVE

advSpawnPt = new OrientedPoint right of curvePoint by globalParameters.ADV_OFFSET

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

## Adversary ##
adversary = new Car at advSpawnPt,
    with blueprint MODEL

param TERMINATION_DIST = 80
param MAX_TIME = 30

require egoSpawnPt can see adversary
terminate when (distance from ego to egoSpawnPt) > globalParameters.TERMINATION_DIST
terminate after globalParameters.MAX_TIME seconds
