description = "Lead vehicle abruptly merges into ego lane with short TTC on a straight highway."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithLeftLane = [s for l in network.lanes for s in l.sections if s.isForward and s._laneToLeft and s._laneToLeft.isForward]

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoInitLane = egoLaneSec.lane
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline

advLaneSec = egoLaneSec._laneToLeft
advInitLane = advLaneSec.lane
anchorPt = advLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following advLaneSec.orientation from anchorPt for Range(15, 25)

param EGO_SPEED = Range(15, 20)
param EGO_BRAKE_THRESHOLD = 15

behavior EgoBehavior(speed, safety_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyObjs(self, safety_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.EGO_BRAKE_THRESHOLD)

param OPT_ADV_SPEED = Range(15, 20)
param OPT_MERGE_TRIGGER_DIST = Range(20, 25)

behavior AdvBehavior(target_speed, target_lane_sec, trigger_dist):
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego) < trigger_dist
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

adv = new Car at advSpawnPt,
    with regionContainedIn advLaneSec,
    with blueprint MODEL,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED, egoLaneSec, globalParameters.OPT_MERGE_TRIGGER_DIST)

INIT_TTC_DIST = [15, 20]
TERM_DIST = 100

require INIT_TTC_DIST[0] <= (distance from ego to adv) <= INIT_TTC_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST