description = "No header settings provided"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(20, 30)

param OPT_ADV_BLOCK_DIST = globalParameters.OPT_LEADING_DIST * 0.3

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
adjLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_BLOCK_DIST

param EGO_SPEED = Range(8, 12)
param SAFETY_DIST = Range(15, 25)
param LANE_CHANGE_DIST = Range(10, 15)

behavior EgoBehavior(ego_speed, safety_dist, lane_change_dist, lane_change_target):
    try:
        do FollowLaneBehavior(target_speed=ego_speed) until withinDistanceToObjsInLane(self, lane_change_dist)
        do LaneChangeBehavior(laneSectionToSwitch=lane_change_target, target_speed=ego_speed)
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when withinDistanceToAnyObjs(self, safety_dist):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with rolename 'hero',
    with behavior EgoBehavior(
        globalParameters.EGO_SPEED,
        globalParameters.SAFETY_DIST,
        globalParameters.LANE_CHANGE_DIST,
        adjLaneSec
    )

param ADV_SPEED = Range(8, 12)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at AdvSpawnPt,
    with heading AdvSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

barrierSpawnPt = new OrientedPoint ahead of egoSpawnPt by globalParameters.OPT_LEADING_DIST * 0.5,
    facing roadDirection
streetBarrier = new Prop at barrierSpawnPt,
    with blueprint 'static.prop.streetbarrier',
    with allowCollisions True

param INIT_DIST = 50
param TERM_DIST = 100

require (distance to intersection) > 50
require (distance from adversary to intersection) > 50
require (distance from streetBarrier to intersection) > 50

terminate when (distance to egoSpawnPt) > 100
terminate when (ego in adjLaneSec) and (distance from ego to streetBarrier) > (distance from egoSpawnPt to LeadingSpawnPt)