description = "Ego vehicle executes lane changes across a multi-lane highway amid dynamic adversary traffic."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*filter(lambda lane:
	all([sec._laneToLeft is not None and sec._laneToRight is not None for sec in lane.sections]),
	network.lanes))
egoSpawnPt = new OrientedPoint in initLane.centerline
laneSec = initLane.sectionAt(egoSpawnPt)
leftSec = laneSec.laneToLeft
rightSec = laneSec.laneToRight
advLeftSpawnPt = new OrientedPoint in leftSec.centerline
advRightSpawnPt = new OrientedPoint in rightSec.centerline

param OPT_EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=rightSec, target_speed=globalParameters.OPT_EGO_SPEED)
    do LaneChangeBehavior(laneSectionToSwitch=rightSec.laneToRight, target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn laneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advLeftSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

behavior AdvStraightBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary2 = new Car at advRightSpawnPt,
	with blueprint MODEL,
	with behavior AdvStraightBehavior()

behavior IntersectingBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=laneSec, target_speed=globalParameters.ADV_SPEED)
    do LaneChangeBehavior(laneSectionToSwitch=leftSec, target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary3 = new Car behind advRightSpawnPt by Range(15, 25),
    facing roadDirection,
    with blueprint MODEL,
    with behavior IntersectingBehavior()

behavior IntersectingLaneChangesBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=laneSec, target_speed=globalParameters.ADV_SPEED)
    do LaneChangeBehavior(laneSectionToSwitch=rightSec, target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary4 = new Car behind advLeftSpawnPt by Range(15, 25),
    facing roadDirection,
    with blueprint MODEL,
    with behavior IntersectingLaneChangesBehavior()