description = "Ego vehicle performs a left lane change between a leading merging vehicle and a trailing vehicle in the target lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(10, 20)
param OPT_TRAILING_DIST = Range(10, 20)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)
egoLaneSec = Uniform(*laneSecsWithLeftLane)
leftLaneSec = egoLaneSec._laneToLeft
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
trailingSpawnPt = new OrientedPoint following roadDirection from leftLanePt for -globalParameters.OPT_TRAILING_DIST

param EGO_SPEED = Range(10, 20)
TIME = 5

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) for TIME seconds
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, target_speed=globalParameters.EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(8, 12)

behavior Adversary1Behavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for TIME seconds
    leftLaneSec = self.laneSection._laneToLeft
    do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary_1 = new Car at leadingSpawnPt,
    with blueprint MODEL,
    with behavior Adversary1Behavior()

param ADV2_SPEED = Range(10, 15)

behavior Adversary2Behavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

adversary_2 = new Car at trailingSpawnPt,
    with blueprint MODEL,
    with behavior Adversary2Behavior()

param TERM_DIST = 150

require (distance from ego to adversary_1) >= 10
require (distance from ego to adversary_1) <= 20
require (distance from ego to adversary_2) >= 10
terminate when (distance to egoSpawnPt) > TERM_DIST