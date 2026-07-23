description = "A motorcycle ahead in the adjacent lane changes into the upper lane, cutting in front of the green vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_MOTO_AHEAD_DIST = Range(20, 40)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

motoLaneSec = egoLaneSec._laneToLeft
motoRefPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_MOTO_AHEAD_DIST
projectPt = motoLaneSec.centerline.project(motoRefPt.position)
motoHeading = motoLaneSec.orientation[projectPt]
motoSpawnPt = new OrientedPoint at projectPt, facing motoHeading

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(8, 12)
param OPT_LC_TRIGGER_DIST = Range(15, 25)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to ego < globalParameters.OPT_LC_TRIGGER_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

AdvAgent = new Motorcycle at motoSpawnPt,
    with heading motoSpawnPt.heading,
    with behavior AdvBehavior()