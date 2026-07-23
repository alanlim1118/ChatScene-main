description = "Ego vehicle approaching a roundabout is cut off by an adversary merging from the left lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithLeftLane = []
for intersection in network.intersections:
    for lane in intersection.incomingLanes:
        for laneSec in lane.sections:
            if (
                laneSec.isForward and
                laneSec._laneToLeft is not None and
                laneSec._laneToLeft.isForward
            ):
                laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
advLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in advLaneSec.centerline

param EGO_SPEED = Range(8, 12)
param SAFETY_DISTANCE = Range(5, 10)

behavior EgoBehavior(speed, safety_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyCars(self, safety_dist):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.EGO_SPEED,
        globalParameters.SAFETY_DISTANCE
    )

param OPT_ADV_SPEED = Range(10, 14)
param OPT_ADV_CUTIN_DIST = Range(15, 25)

behavior AdvBehavior(adv_speed, cutin_dist):
    do FollowLaneBehavior(target_speed=adv_speed) until (distance from self to ego < cutin_dist)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=adv_speed)
    do FollowLaneBehavior(target_speed=adv_speed)

AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn advLaneSec,
    with behavior AdvBehavior(
        globalParameters.OPT_ADV_SPEED,
        globalParameters.OPT_ADV_CUTIN_DIST
    )