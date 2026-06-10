description = "Ego bypasses a parked car, but an oncoming vehicle suddenly turns into its path, requiring evasive action."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_PARKED_DIST = Range(30, 40)
param OPT_ONCOMING_DIST = Range(60, 80)

laneSecsWithOncoming = []
for lane in network.lanes:
    if lane.centerline.length > 100:
        for laneSec in lane.sections:
            if laneSec._laneToLeft is not None:
                if laneSec._laneToLeft.isForward != laneSec.isForward:
                    laneSecsWithOncoming.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithOncoming)
oncomingLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint on egoLaneSec.centerline
parkedCarSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_PARKED_DIST

oncomingSpawnPt = new OrientedPoint on oncomingLaneSec.centerline
require (distance from oncomingSpawnPt to egoSpawnPt) >= globalParameters.OPT_ONCOMING_DIST
require (distance from oncomingSpawnPt to egoSpawnPt) <= globalParameters.OPT_ONCOMING_DIST + 15
require (relative heading of oncomingSpawnPt from egoSpawnPt) < 0.2

egoTrajectory = [egoLaneSec.lane]
oncomingTrajectory = [oncomingLaneSec.lane]

param OPT_EGO_SPEED = Range(6, 10)
param OPT_BYPASS_DIST = 15
param OPT_EVASIVE_DIST = 25

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to parkedCar) < globalParameters.OPT_BYPASS_DIST
        do LaneChangeBehavior(laneSectionToSwitch=oncomingLaneSec, is_oppositeTraffic=True, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to parkedCar) > globalParameters.OPT_BYPASS_DIST
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to oncomingCar) < globalParameters.OPT_EVASIVE_DIST:
        take SetBrakeAction(1.0)
        take SetThrottleAction(0.0)
        wait until (distance from self to oncomingCar) > globalParameters.OPT_EVASIVE_DIST
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

parkedCar = new Car at parkedCarSpawnPt,
    with blueprint MODEL,
    with regionContainedIn None

param OPT_ONCOMING_SPEED = Range(7, 10)
param OPT_TURN_TRIGGER_DIST = Range(20, 30)

behavior OncomingBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ONCOMING_SPEED) until (distance from self to ego) < globalParameters.OPT_TURN_TRIGGER_DIST
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=True, target_speed=globalParameters.OPT_ONCOMING_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ONCOMING_SPEED)

oncomingCar = new Car at oncomingSpawnPt,
    with blueprint MODEL,
    with regionContainedIn oncomingLaneSec,
    with behavior OncomingBehavior()

TERM_DIST = 100

require 30 <= (distance from egoSpawnPt to parkedCarSpawnPt) <= 40
terminate when (distance from ego to egoSpawnPt) > TERM_DIST