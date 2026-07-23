description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather ClearNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
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

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=10)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn None,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

require 30 <= (distance from egoSpawnPt to intersection) <= 50
require 5 <= (distance from advSpawnPt to intersection) <= 15
terminate when (distance from ego to intersection) > 60