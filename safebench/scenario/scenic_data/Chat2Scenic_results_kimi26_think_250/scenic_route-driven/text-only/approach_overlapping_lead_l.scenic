description = "Ego vehicle travels straight within its lane and approaches a leading adversarial object overlapping from the left."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
leftLaneSec = egoLaneSec._laneToLeft
adjLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
laneOffset = egoSpawnPt.position - adjLanePt
boundaryPt = adjLanePt + laneOffset * 0.5
advSpawnPt = new OrientedPoint following leftLaneSec.orientation from boundaryPt for Range(10, 30)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

adversary = new Car at advSpawnPt,
    with blueprint MODEL