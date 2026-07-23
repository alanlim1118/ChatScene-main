description = "A motorcycle ahead in the adjacent lane changes into the upper lane, cutting in front of the green vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_MOTO_AHEAD_DIST = Range(20, 40)

laneSecsWithLeftForwardLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward == laneSec.isForward:
                laneSecsWithLeftForwardLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftForwardLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

motoLaneSec = egoLaneSec._laneToLeft
motoRefPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_MOTO_AHEAD_DIST
projectPt = motoLaneSec.centerline.project(motoRefPt.position)
motoHeading = motoLaneSec.orientation[projectPt]
motoSpawnPt = new OrientedPoint at projectPt, facing motoHeading

param EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(8, 12)
param OPT_LC_TRIGGER_DIST = Range(15, 25)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to ego < globalParameters.OPT_LC_TRIGGER_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

AdvAgent = new Motorcycle at motoSpawnPt,
    with heading motoSpawnPt.heading,
    with behavior AdvBehavior()