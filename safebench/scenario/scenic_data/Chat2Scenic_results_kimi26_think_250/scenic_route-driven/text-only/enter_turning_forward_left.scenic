description = "Ego vehicle travels straight in the middle lane while an adversary cuts ahead from the left lane toward the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
leftLaneSec = egoLaneSec._laneToLeft

param OPT_ADV_AHEAD_DIST = Range(8, 20)
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_ADV_AHEAD_DIST

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(8, 14)

behavior AdversaryBehavior():
    do LaneChangeBehavior(egoLaneSec, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at AdvSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

require 8 <= (distance from egoSpawnPt to AdvSpawnPt) <= 25