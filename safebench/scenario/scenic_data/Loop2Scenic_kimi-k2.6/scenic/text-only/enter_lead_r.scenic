description = "No header settings provided"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_BLOCK_DIST = Range(20, 30)
param OPT_LEADING_DIST = Range(10, 20)

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
adjLaneSec = egoLaneSec._laneToRight

LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_BLOCK_DIST

param EGO_SPEED = Range(10, 15)

behavior EgoBehavior(target_speed, target_lane):
	do FollowLaneBehavior(target_speed=target_speed) for Range(2, 4) seconds
	do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
	do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn egoLaneSec,
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.EGO_SPEED, adjLaneSec)

param OPT_ADV_SPEED = Range(8, 12)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    terminate

adversary = new Car at AdvSpawnPt,
    with heading AdvSpawnPt.heading,
    with regionContainedIn adjLaneSec,
    with behavior AdvBehavior()

param TERM_DIST = 30

require (distance from egoSpawnPt to AdvSpawnPt) >= 20
terminate when (distance from ego to AdvSpawnPt) > (distance from AdvSpawnPt to egoSpawnPt) + globalParameters.OPT_LEADING_DIST + globalParameters.TERM_DIST