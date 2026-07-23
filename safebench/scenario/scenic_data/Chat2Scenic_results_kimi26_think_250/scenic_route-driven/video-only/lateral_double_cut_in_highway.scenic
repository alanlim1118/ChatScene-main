description = "Ego vehicle performs emergency braking on a highway to avoid a rear-end collision after a black SUV cuts in and stops abruptly."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)
advLane = network.laneSectionAt(egoSpawnPt)._laneToRight.lane
advSpawnPt = new OrientedPoint in advLane.centerline

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(8, 12)

behavior AdvBehavior():
    leftLaneSec = self.laneSection._laneToLeft
    do LaneChangeBehavior(
            laneSectionToSwitch=leftLaneSec,
            target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=0)

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdvBehavior()

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 60
terminate when (distance from ego to adversary) <= 5