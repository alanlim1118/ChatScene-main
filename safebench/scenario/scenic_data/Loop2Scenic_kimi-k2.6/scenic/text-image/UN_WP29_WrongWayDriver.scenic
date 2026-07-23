description = "Using map ../../maps/Town04.xodr with carla map Town04 and weather ClearNoon"
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
adjLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in adjLaneSec.centerline

param EGO_SPEED = Range(9, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero',
	with behavior EgoBehavior()

param ADV_SPEED = Range(9, 11)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, is_oppositeTraffic=True)

adversary = new Car at advSpawnPt,
    facing roadDirection,
    with regionContainedIn None,
    with behavior AdvBehavior()

param PASS_DIST = Range(30, 50)

require (distance from ego to advSpawnPt) >= 20
require (distance from adversary to egoSpawnPt) >= 20

terminate when (distance from ego to advSpawnPt) > globalParameters.PASS_DIST and (distance from adversary to egoSpawnPt) > globalParameters.PASS_DIST