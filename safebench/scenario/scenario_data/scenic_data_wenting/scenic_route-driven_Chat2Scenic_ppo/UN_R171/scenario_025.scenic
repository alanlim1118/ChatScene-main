description = "Ego vehicle avoids rear-end collision with slower vehicle performing a lane change into its path."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLaneSec = network.laneSectionAt(egoSpawnPt)

advLaneSec = egoLaneSec._laneToRight
require advLaneSec is not None
require advLaneSec.isForward

egoInitLane = egoLaneSec.lane
advInitLane = advLaneSec.lane

refPtOnAdvLane = advLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following advLaneSec.orientation from refPtOnAdvLane for Range(15, 25)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(10, 15)
param OPT_ADV_THRESHOLD = Range(20, 30)

behavior AdversaryBehavior(speed, threshold):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego) < threshold
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_THRESHOLD)

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 25
terminate when (distance from ego to egoSpawnPt) > 100
terminate after 30 seconds
