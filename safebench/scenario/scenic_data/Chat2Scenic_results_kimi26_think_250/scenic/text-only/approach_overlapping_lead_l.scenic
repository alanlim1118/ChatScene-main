description = "Ego vehicle travels straight within its lane and approaches a leading adversarial object overlapping from the left."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
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
leftLaneSec = egoLaneSec._laneToLeft
adjLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
laneOffset = egoSpawnPt.position - adjLanePt
boundaryPt = adjLanePt + laneOffset * 0.5
advSpawnPt = new OrientedPoint following leftLaneSec.orientation from boundaryPt for Range(10, 30)

param OPT_EGO_SPEED = Range(5, 8)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

adversary = new Car at advSpawnPt,
    with blueprint MODEL