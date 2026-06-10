description = "Ego vehicle travels straight at constant speed with minimal lateral offset before approaching a stationary vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
advSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(20, 30)

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL

require (distance from egoSpawnPt to initLane.centerline) <= 0.5
require 20 <= (distance from egoSpawnPt to advSpawnPt) <= 30
terminate after 10 seconds