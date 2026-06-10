description = "Ego vehicle drifts into an adjacent vehicle while driving straight on a high-speed urban road."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

adjLaneSec = egoLaneSec._laneToLeft
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint at adjLanePt, facing egoSpawnPt.heading

param EGO_SPEED = Range(15, 20)
param TIME_TO_DRIFT = Range(2, 4)

behavior EgoBehavior(speed, target_lane):
    do FollowLaneBehavior(target_speed=speed) for globalParameters.TIME_TO_DRIFT seconds
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, adjLaneSec)

param ADV_SPEED = Range(15, 20)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

INIT_DIST = 50
TERM_DIST = 120

require (distance from egoSpawnPt to intersection) > INIT_DIST
terminate when (distance from ego to egoSpawnPt) > TERM_DIST