description = "Subject vehicle performs a lane change to avoid a lane reduction signboard."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

egoInitLane = network.laneAt(egoSpawnPt.position)
egoSection = network.laneSectionAt(egoSpawnPt)

warningSign = new TrafficWarning following roadDirection from egoSpawnPt for Range(20, 30)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)
param OPT_ADV_DIST = Range(10, 15)

behavior AdversaryBehavior(speed):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to warningSign < 15)
    do LaneChangeBehavior(laneSectionToSwitch=(egoSection._laneToLeft if egoSection._laneToLeft else egoSection._laneToRight), target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

adversary = new Car following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_DIST,
    with blueprint MODEL,
    with behavior AdversaryBehavior(globalParameters.OPT_ADV_SPEED)

require 10 <= (distance from ego to adversary) <= 15
require 20 <= (distance from ego to warningSign) <= 30
terminate when (distance from ego to egoSpawnPt) > 60
terminate after 500 steps
