description = "Ego vehicle changes lane to the right as an adversarial vehicle cuts through from the left lane to the same right lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_DIST = Range(20, 40)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

egoLane = egoLaneSec.lane
leftLane = leftLaneSec.lane
rightLane = rightLaneSec.lane

advBasePt = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advBasePt for globalParameters.OPT_ADV_DIST

egoTrajectory = [egoLane, rightLane]
advTrajectory = [leftLane, rightLane]

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(8, 12)

behavior AdversaryBehavior():
    do LaneChangeBehavior(
        laneSectionToSwitch=egoLaneSec,
        target_speed=globalParameters.OPT_ADV_SPEED)
    do LaneChangeBehavior(
        laneSectionToSwitch=rightLaneSec,
        target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with regionContainedIn None,
    with behavior AdversaryBehavior()