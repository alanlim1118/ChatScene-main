description = "Ego bypasses a parked car, encounters a sudden pedestrian in the opposite lane, requiring sharp braking or evasive steering."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param PARKED_CAR_DIST = Range(20, 30)
param PED_LON_OFFSET = Range(3, 8)
param LANE_WIDTH = 3.5

oppositeLaneSections = []
for lane in network.lanes:
    for section in lane.sections:
        if section._laneToLeft is not None and section._laneToLeft.isForward != section.isForward:
            oppositeLaneSections.append(section)

egoSection = Uniform(*oppositeLaneSections)
egoSpawnPt = new OrientedPoint in egoSection.centerline

parkedCarSpawnPt = new OrientedPoint following egoSection.orientation from egoSpawnPt for globalParameters.PARKED_CAR_DIST

pedIntermediatePt = new OrientedPoint following egoSection.orientation from egoSpawnPt for (globalParameters.PARKED_CAR_DIST + globalParameters.PED_LON_OFFSET)
pedSpawnPt = new OrientedPoint left of pedIntermediatePt by globalParameters.LANE_WIDTH

egoTrajectory = [egoSection.lane]

param EGO_SPEED = Range(5, 8)
param BYPASS_THRESHOLD = 15
param PED_AVOID_DIST = 10

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until (distance to parkedCarSpawnPt) < globalParameters.BYPASS_THRESHOLD
    try:
        do LaneChangeBehavior(laneSectionToSwitch=egoSection._laneToLeft, is_oppositeTraffic=True, target_speed=globalParameters.EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED, is_oppositeTraffic=True)
    interrupt when withinDistanceToAnyPedestrians(self, globalParameters.PED_AVOID_DIST):
        take SetBrakeAction(1.0)
        terminate

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

parkedCar = new Car at parkedCarSpawnPt,
    with blueprint MODEL,
    with heading parkedCarSpawnPt.heading,
    with regionContainedIn None

param OPT_ADV_SPEED = Range(1.2, 1.8)
param OPT_ADV_DISTANCE = Range(18, 24)
param OPT_STOP_DISTANCE = 0.8

behavior CrossAndStopBehavior(actor_reference, adv_speed, adv_distance, stop_reference, stop_distance):
    do CrossingBehavior(actor_reference, adv_speed, adv_distance) until (distance from self to stop_reference <= stop_distance)
    take SetWalkingSpeedAction(0)

ped = new Pedestrian at pedSpawnPt,
    facing toward pedIntermediatePt,
    with regionContainedIn None,
    with behavior CrossAndStopBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE, pedIntermediatePt, globalParameters.OPT_STOP_DISTANCE)

terminate when ego intersects parkedCar
terminate when ego intersects ped
terminate when (distance from ego to egoSpawnPt) > (globalParameters.PARKED_CAR_DIST + globalParameters.PED_LON_OFFSET + 10)