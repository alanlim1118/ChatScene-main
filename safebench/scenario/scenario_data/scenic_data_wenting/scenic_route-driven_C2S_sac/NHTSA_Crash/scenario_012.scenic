description = "Vehicle A performs an unsafe lane change into Vehicle B's lane, resulting in a collision."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoSection = network.laneSectionAt(egoSpawnPt)

param OPT_DISTANCE = Range(10, 20)

targetPt = new OrientedPoint following egoSection.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE
sideIsLeft = egoSection._laneToLeft is not None
advSpawnPt = new OrientedPoint left of targetPt by 3.5 if sideIsLeft else new OrientedPoint right of targetPt by 3.5

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(8, 12)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

require sideIsLeft
require 10 <= (distance from egoSpawnPt to advSpawnPt) <= 25
terminate when ego intersects adversary
terminate after 20 seconds
