description = "Ego vehicle brakes to avoid a perpendicularly crossing bicycle emerging from an obstructed view."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.diamondback.century'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)

param OPT_DIST_AHEAD = Range(30, 45)

IntPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_DIST_AHEAD

bikeSpawnPt = new OrientedPoint right of IntPt by Range(8, 12)
bikeSpawnPt = new OrientedPoint at bikeSpawnPt, facing (egoSpawnPt.heading - 90 deg)

blockerIntPt = new OrientedPoint following roadDirection from egoSpawnPt for (globalParameters.OPT_DIST_AHEAD - 10)
blockerSpawnPt = new OrientedPoint right of blockerIntPt by 3.5

param EGO_MODEL = 'vehicle.lincoln.mkz_2017'

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint globalParameters.EGO_MODEL

param BICYCLE_MIN_SPEED = 1.5
param BICYCLE_THRESHOLD = 25

behavior BicycleBehavior():
    do CrossingBehavior(ego, globalParameters.BICYCLE_MIN_SPEED, globalParameters.BICYCLE_THRESHOLD)

obstruction = new Car at blockerSpawnPt

advBicycle = new Bicycle at bikeSpawnPt,
    with blueprint MODEL,
    with behavior BicycleBehavior(),
    with regionContainedIn None

param TERM_DIST = 10

require 30 <= (distance from egoSpawnPt to IntPt) <= 45
require 8 <= (distance from bikeSpawnPt to IntPt) <= 12

terminate when (distance from ego to egoSpawnPt) > (globalParameters.OPT_DIST_AHEAD + globalParameters.TERM_DIST)
