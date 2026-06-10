description = "Ego vehicle departs a parked position at night in an urban area and collides with an object on the road shoulder/parking lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLaneSec = network.laneSectionAt(egoSpawnPt)

require egoLaneSec._laneToRight is None and egoLaneSec._laneToLeft is not None

param OPT_propDistance = Range(20, 30)
propSpawnPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_propDistance

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

prop = new Trash at propSpawnPt

terminate when ego intersects prop
