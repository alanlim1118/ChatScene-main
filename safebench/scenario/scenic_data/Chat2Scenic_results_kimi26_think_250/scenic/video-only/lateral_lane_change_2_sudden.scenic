description = "A taxi abruptly cuts into the ego lane on a city road, forcing heavy braking."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_AHEAD_DIST = Range(5, 15)

laneSecsWithLeftAndRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithLeftAndRight.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftAndRight)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

advLaneSec = egoLaneSec._laneToRight
adjLanePt = advLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_AHEAD_DIST

param OPT_EGO_SPEED = Range(8, 12)
param OPT_BRAKE_DISTANCE = Range(5, 8)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(ego, globalParameters.OPT_BRAKE_DISTANCE)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(2, 5)
param ADV_FORWARD_TIME = Range(1, 3)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for globalParameters.ADV_FORWARD_TIME seconds
    leftLaneSec = self.laneSection._laneToLeft
    do LaneChangeBehavior(
        laneSectionToSwitch=leftLaneSec,
        target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at AdvSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()