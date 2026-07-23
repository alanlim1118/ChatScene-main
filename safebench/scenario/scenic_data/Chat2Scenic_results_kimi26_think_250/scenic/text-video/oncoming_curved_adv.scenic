description = "Ego vehicle collides head-on with a white microvan drifting into its lane from a blind corner on a curved rural road."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

forwardLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec.road.backwardLanes is not None:
            forwardLaneSecs.append(laneSec)
egoLaneSec = Uniform(*forwardLaneSecs)
advLane = Uniform(*egoLaneSec.road.backwardLanes.lanes)
advLaneSec = Uniform(*advLane.sections)

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in advLaneSec.centerline

param EGO_SPEED = Range(8, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(8, 12)

behavior AdvBehavior():
	do LaneChangeBehavior(egoLaneSec, is_oppositeTraffic=True, target_speed=globalParameters.ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, is_oppositeTraffic=True)

adversarial = new Car at advSpawnPt,
	facing advSpawnPt.heading,
	with regionContainedIn None,
	with behavior AdvBehavior()

param INIT_DIST = [40, 80]

require globalParameters.INIT_DIST[0] <= (distance from ego to adversarial) <= globalParameters.INIT_DIST[1]
terminate when ego intersects adversarial