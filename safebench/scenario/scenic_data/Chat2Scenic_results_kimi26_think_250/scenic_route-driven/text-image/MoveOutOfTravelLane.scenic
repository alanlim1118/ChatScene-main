description = "Ego vehicle exits the travel lane to park in a gap between two stationary vehicles on a straight urban street."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_PARKED_OFFSET = Range(15, 25)
param OPT_PARKED_GAP = Range(10, 15)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)
rightLaneSec = egoLaneSec._laneToRight

rearBasePos = rightLaneSec.centerline.project(egoSpawnPt.position)
parkedRearSpawnPt = new OrientedPoint following roadDirection from rearBasePos for globalParameters.OPT_PARKED_OFFSET
parkedFrontSpawnPt = new OrientedPoint following roadDirection from parkedRearSpawnPt for globalParameters.OPT_PARKED_GAP

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

adversary = new Car at parkedRearSpawnPt,
    with blueprint MODEL

adversary_front = new Car at parkedFrontSpawnPt,
    with blueprint MODEL