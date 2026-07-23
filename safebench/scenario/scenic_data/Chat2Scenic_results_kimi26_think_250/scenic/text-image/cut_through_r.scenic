description = "Ego vehicle travels straight in the center lane while an adversarial vehicle cuts across from the right lane to the left lane in front of it."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithRightLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

rightLaneSec = egoLaneSec._laneToRight
leftLaneSec = egoLaneSec._laneToLeft

advRefPt = rightLaneSec.centerline.project(egoSpawnPt.position)
param OPT_ADV_START_DIST = Range(-10, -5)
advSpawnPt = new OrientedPoint following egoLaneSec.orientation from advRefPt for globalParameters.OPT_ADV_START_DIST

rightLane = rightLaneSec.lane
centerLane = egoLaneSec.lane
leftLane = leftLaneSec.lane
advTrajectory = [rightLane, centerLane, leftLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param OPT_ADV_SPEED = globalParameters.EGO_SPEED + Range(2, 5)

behavior AdvBehavior(target_speed):
    do FollowTrajectoryBehavior(target_speed, advTrajectory)
    terminate

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with behavior AdvBehavior(globalParameters.OPT_ADV_SPEED)