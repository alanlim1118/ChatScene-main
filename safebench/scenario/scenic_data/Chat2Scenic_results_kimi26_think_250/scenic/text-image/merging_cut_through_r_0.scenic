description = "Ego vehicle changes lane to the right as an adversarial vehicle cuts through from the left lane to the same right lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_DIST = Range(20, 40)

laneSecsWithLeftAndRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithLeftAndRight.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftAndRight)
leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

egoLane = egoLaneSec.lane
leftLane = leftLaneSec.lane
rightLane = rightLaneSec.lane

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advBasePt = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advBasePt for globalParameters.OPT_ADV_DIST

egoTrajectory = [egoLane, rightLane]
advTrajectory = [leftLane, rightLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=rightLaneSec, target_speed=globalParameters.EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(8, 12)

behavior AdversaryBehavior():
    do LaneChangeBehavior(
        laneSectionToSwitch=egoLaneSec,
        target_speed=globalParameters.ADV_SPEED)
    do LaneChangeBehavior(
        laneSectionToSwitch=rightLaneSec,
        target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with regionContainedIn None,
    with behavior AdversaryBehavior()