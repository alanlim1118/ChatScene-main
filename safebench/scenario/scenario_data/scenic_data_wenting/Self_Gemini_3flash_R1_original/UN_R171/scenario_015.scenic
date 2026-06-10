description = "Ego approaches a slower vehicle ahead with a significant lateral offset within its lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
pointAhead = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(20, 30)
advSpawnPt = new OrientedPoint left of pointAhead by Uniform(Range(0.8, 1.2), Range(-1.2, -0.8))

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

param ADV_SPEED = Range(3, 5)

behavior AdversaryBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(globalParameters.ADV_SPEED)

require 0.8 <= (distance from advSpawnPt to pointAhead) <= 1.2
terminate when (distance from ego to adversary) > 50
terminate after 40 seconds