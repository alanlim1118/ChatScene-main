description = "Ego vehicle encounters a lane-blocking obstacle and changes lanes into oncoming traffic to avoid it."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_BLOCKER_DISTANCE = Range(20, 30)

laneSecsWithLeftOpposite = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithLeftOpposite.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftOpposite)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

blockerSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_DISTANCE

advLaneSec = egoLaneSec._laneToLeft
advSpawnPt = new OrientedPoint in advLaneSec.centerline

param OPT_EGO_SPEED = Range(7, 10)
LANE_CHANGE_DISTANCE = 15

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to blockerSpawnPt) < LANE_CHANGE_DISTANCE
    do LaneChangeBehavior(laneSectionToSwitch=advLaneSec, is_oppositeTraffic=True, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed, is_oppositeTraffic=True)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED)

adversary = new Car at blockerSpawnPt

param OPT_ADV_SPEED = Range(7, 10)

behavior AdvBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)

param TERM_DIST = 50

terminate when (distance to egoSpawnPt) > TERM_DIST