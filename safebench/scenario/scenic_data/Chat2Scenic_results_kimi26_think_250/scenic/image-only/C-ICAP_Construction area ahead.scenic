description = "Green vehicle approaches a construction zone with diagonal red barriers and an iron sheet wall blocking the left lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsLeftmost = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is None and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsLeftmost.append(laneSec)

egoLaneSec = Uniform(*laneSecsLeftmost)
adjLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
barrierBasePt = new OrientedPoint following roadDirection from egoSpawnPt for Range(25, 35)
barrier1SpawnPt = new OrientedPoint left of barrierBasePt by Range(0.5, 1.5)
barrier2BasePt = new OrientedPoint following roadDirection from barrierBasePt for Range(3, 6)
barrier2SpawnPt = new OrientedPoint right of barrier2BasePt by Range(0.5, 1.5)
ironPlateSpawnPt = new OrientedPoint following roadDirection from barrier2BasePt for Range(5, 10)

param OPT_EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

BARRIER = 'static.prop.streetbarrier'

barrier = new Barrier at barrier1SpawnPt,
    with blueprint BARRIER,
    facing -90 deg relative to barrier1SpawnPt.heading

ironPlate = new IronPlate at ironPlateSpawnPt