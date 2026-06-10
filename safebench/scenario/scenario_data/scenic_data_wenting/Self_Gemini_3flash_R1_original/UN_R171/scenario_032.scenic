description = "Ego vehicle brakes to avoid a perpendicularly crossing bicycle emerging from an obstructed view."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.diamondback.century'
param weather = 'ClearNoon'

egoLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoLane.centerline

param distAhead = Range(30, 45)

# Intersection point on the road where the bicycle will cross
IntPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.distAhead

# Position the bicycle to the right of the road, oriented perpendicularly towards the ego's path
bikeSpawnPt = new OrientedPoint right of IntPt by Range(8, 12)
bikeSpawnPt = new OrientedPoint at bikeSpawnPt, facing (egoSpawnPt.heading - 90 deg)

# Position an obstruction point between the ego and the bicycle to block the line of sight
blockerIntPt = new OrientedPoint following roadDirection from egoSpawnPt for (globalParameters.distAhead - 10)
blockerSpawnPt = new OrientedPoint right of blockerIntPt by 3.5

param EGO_SPEED = Range(10, 15)
param EGO_BRAKE = 1.0
param SAFETY_DIST = Range(15, 20)
param EGO_MODEL = 'vehicle.lincoln.mkz_2017'

behavior EgoBehavior(speed, brake_val, safety_dist):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint globalParameters.EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.EGO_BRAKE, globalParameters.SAFETY_DIST)

param BICYCLE_MIN_SPEED = 1.5
param BICYCLE_THRESHOLD = 25

behavior BicycleBehavior():
    do CrossingBehavior(ego, globalParameters.BICYCLE_MIN_SPEED, globalParameters.BICYCLE_THRESHOLD)

# Static car to act as the obstruction blocking the ego's view of the bicycle
obstruction = new Car at blockerSpawnPt

# Adversarial bicycle that emerges and crosses perpendicularly
advBicycle = new Bicycle at bikeSpawnPt,
    with blueprint MODEL,
    with behavior BicycleBehavior(),
    with regionContainedIn None

param TERM_DIST = 10

require 30 <= (distance from egoSpawnPt to IntPt) <= 45
require 8 <= (distance from bikeSpawnPt to IntPt) <= 12

terminate when (distance from ego to egoSpawnPt) > (globalParameters.distAhead + globalParameters.TERM_DIST)