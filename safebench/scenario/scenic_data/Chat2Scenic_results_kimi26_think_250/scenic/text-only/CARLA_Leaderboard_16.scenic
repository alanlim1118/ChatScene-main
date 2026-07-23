description = "Ego vehicle must brake or maneuver to avoid a slow moving hazard blocking part of the lane next to oncoming traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithOncomingLeft = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and not laneSec._laneToLeft.isForward:
            laneSecsWithOncomingLeft.append(laneSec)
egoLaneSec = Uniform(*laneSecsWithOncomingLeft)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(10, 40)

param EGO_SPEED = Range(8, 12)
param EGO_BRAKE = Range(0.6, 1.0)
AVOID_DIST = 12

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, AVOID_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(2, 4)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adv = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdvBehavior()