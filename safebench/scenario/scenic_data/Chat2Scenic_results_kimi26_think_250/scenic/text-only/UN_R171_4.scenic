description = "Ego vehicle aborts lane change upon detecting a rapidly approaching motorcycle in the adjacent lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_CAR_DISTANCE = Range(15, 30)
param OPT_MOTO_DISTANCE = Range(5, 15)

laneSecsMulti = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToRight is not None:
            if laneSec._laneToRight.isForward == laneSec.isForward:
                laneSecsMulti.append(laneSec)

egoLaneSec = Uniform(*laneSecsMulti)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

carSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_CAR_DISTANCE

adjLaneSec = egoLaneSec._laneToRight
adjRefPt = adjLaneSec.centerline.project(egoSpawnPt.position)
motoRefPt = new OrientedPoint at adjRefPt, facing egoSpawnPt.heading
motoSpawnPt = new OrientedPoint behind motoRefPt by globalParameters.OPT_MOTO_DISTANCE

param OPT_EGO_SPEED = Range(6, 10)
param OPT_PASS_DIST = Range(8, 15)
param OPT_MOTO_DETECT_DIST = Range(3, 5)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to carSpawnPt < globalParameters.OPT_PASS_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=adjLaneSec, target_speed=globalParameters.OPT_EGO_SPEED) until (withinDistanceToAnyCars(self, globalParameters.OPT_MOTO_DETECT_DIST))
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at carSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

param OPT_MOTO_SPEED = Range(14, 18)

behavior MotoFilterBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_MOTO_SPEED)

motorcycle = new Motorcycle at motoSpawnPt,
    facing motoSpawnPt.heading,
    with regionContainedIn adjLaneSec,
    with behavior MotoFilterBehavior()

require 15 <= (distance from ego to adversary) <= 30
require 5 <= (distance from ego to motorcycle) <= 15
terminate when (ego in egoLaneSec) and (distance from ego to egoSpawnPt) > globalParameters.OPT_CAR_DISTANCE