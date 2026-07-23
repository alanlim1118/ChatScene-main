description = "Ego vehicle approaches a laterally moving object entering from the right forward with converging paths in the right lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
advLaneSec = egoLaneSec._laneToRight
basePt = advLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following advLaneSec.lane.orientation from basePt for Range(10, 30)

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(5, 8)

behavior AdversaryBehavior():
    leftLaneSec = self.laneSection._laneToLeft
    do LaneChangeBehavior(
        laneSectionToSwitch=leftLaneSec,
        target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

TERM_DIST = 100

require 10 <= (distance from ego to adversary) <= 35
terminate when (distance to egoSpawnPt) > TERM_DIST