description = "Ego vehicle departs a parked position at night in an urban area and collides with an object on the road shoulder/parking lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSections = []
for lane in network.lanes:
    for section in lane.sections:
        if section._laneToRight is None and section._laneToLeft is not None:
            laneSections.append(section)

egoLaneSec = Uniform(*laneSections)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

propDistance = 30
propSpawnPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for propDistance

param OPT_EGO_SPEED = Range(5, 10)

behavior EgoBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED)

prop = new Trash at propSpawnPt

terminate when ego intersects prop