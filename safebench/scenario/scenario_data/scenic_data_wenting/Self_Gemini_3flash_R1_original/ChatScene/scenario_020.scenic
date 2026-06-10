description = "Ego bypasses a parked car in its lane using the oncoming lane, but an accelerating oncoming vehicle creates a collision risk, forcing a rapid decision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_PARKED_DIST = Range(20, 30)
param OPT_ONCOMING_DIST = Range(50, 70)

laneSecsWithOppositeLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithOppositeLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithOppositeLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

parkedCarSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_PARKED_DIST

oppLaneSec = egoLaneSec._laneToLeft
oncomingTargetPt = new OrientedPoint following roadDirection from parkedCarSpawnPt for globalParameters.OPT_ONCOMING_DIST
oncomingSpawnPt = new OrientedPoint on oppLaneSec.centerline, at oppLaneSec.centerline.project(oncomingTargetPt.position)

egoTrajectory = [egoLaneSec.lane, oppLaneSec.lane]
oncomingTrajectory = [oppLaneSec.lane]

param OPT_EGO_SPEED = Range(5, 8)
param OPT_BYPASS_DIST = 15
param OPT_SAFETY_DIST = 20

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to parkedCarSpawnPt < globalParameters.OPT_BYPASS_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=oppLaneSec, is_oppositeTraffic=True, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to parkedCarSpawnPt > 10)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

parkedCar = new Car at parkedCarSpawnPt,
    with blueprint MODEL,
    with heading parkedCarSpawnPt.heading,
    with regionContainedIn None

param ADV_INIT_SPEED = Range(5, 8)
param ADV_END_SPEED = Range(15, 20)
param ADV_TRIGGER_DIST = Range(40, 50)

behavior OncomingBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_INIT_SPEED, trajectory=oncomingTrajectory) until (distance from self to ego) < globalParameters.ADV_TRIGGER_DIST
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_END_SPEED, trajectory=oncomingTrajectory)

oncomingCar = new Car at oncomingSpawnPt,
    with blueprint MODEL,
    with behavior OncomingBehavior()

require (distance from ego to parkedCar) >= 20
require (distance from ego to parkedCar) <= 30
require (distance from ego to oncomingCar) >= 70
require (distance from ego to oncomingCar) <= 100

terminate when (distance from ego to parkedCar) > 40