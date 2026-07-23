description = "Using map ../../maps/Town05.xodr with carla map Town05 and weather ClearNoon"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (
            laneSec.isForward and
            laneSec._laneToRight is not None and
            laneSec._laneToRight.isForward
        ):
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
rightLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in rightLaneSec.centerline

param EGO_SPEED = Range(9, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero',
	with behavior EgoBehavior()

param ADV_SPEED = Range(5, 7)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior([egoLaneSec.lane, rightLaneSec.lane])

require (distance from egoSpawnPt to intersection) > 30
require (distance from advSpawnPt to intersection) > 30
terminate when (ego intersects adversary)