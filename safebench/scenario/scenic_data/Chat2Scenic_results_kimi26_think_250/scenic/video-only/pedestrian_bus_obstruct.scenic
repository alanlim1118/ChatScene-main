description = "Ego vehicle overtakes a stopped bus on a rural road as a pedestrian emerges from the blind spot, causing a collision."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

param OPT_GEO_BLOCKER_Y_DISTANCE = Range(20, 30)
param OPT_GEO_X_DISTANCE = Range(2, 4)
param OPT_GEO_Y_DISTANCE = Range(2, 6)
param OPT_GEO_GUARD_Y_DISTANCE = Range(10, 20)
param OPT_GEO_GUARD_X_DISTANCE = Range(3, 5)

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

npcIntPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_Y_DISTANCE
npcSpawnPt = new OrientedPoint right of npcIntPt by globalParameters.OPT_GEO_X_DISTANCE

pedSpawnPt = new OrientedPoint ahead of npcSpawnPt by globalParameters.OPT_GEO_Y_DISTANCE

oncomingLaneSec = egoLaneSec._laneToLeft
oncomingSpawnPt = new OrientedPoint in oncomingLaneSec.centerline

guardIntPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_GEO_GUARD_Y_DISTANCE
guardPedSpawnPt = new OrientedPoint right of guardIntPt by globalParameters.OPT_GEO_GUARD_X_DISTANCE

param OPT_EGO_SPEED = Range(8, 12)
param OPT_EGO_OVERTAKE_DIST = Range(10, 15)
param OPT_EGO_SAFETY_DIST = Range(5, 8)

behavior EgoBehavior(ego_speed, overtake_threshold, safety_distance, lane_change_target):
    try:
        do FollowLaneBehavior(target_speed=ego_speed) until withinDistanceToObjsInLane(self, overtake_threshold)
        do LaneChangeBehavior(laneSectionToSwitch=lane_change_target, target_speed=ego_speed, is_oppositeTraffic=True)
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when withinDistanceToAnyPedestrians(self, safety_distance):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_EGO_OVERTAKE_DIST,
        globalParameters.OPT_EGO_SAFETY_DIST,
        oncomingLaneSec
    )

npc = new NPCCar at npcSpawnPt,
    with regionContainedIn None

param OPT_PED_MIN_SPEED = Range(0.5, 1.5)
param OPT_PED_THRESHOLD = Range(10, 15)

behavior PedestrianCrossingBehavior():
    do CrossingBehavior(reference_actor=ego, min_speed=globalParameters.OPT_PED_MIN_SPEED, threshold=globalParameters.OPT_PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
    facing 90 deg relative to pedSpawnPt.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossingBehavior()

param OPT_ONCOMING_SPEED = Range(8, 12)

behavior OncomingCarBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

oncomingCar = new Car at oncomingSpawnPt,
    with regionContainedIn None,
    with behavior OncomingCarBehavior(globalParameters.OPT_ONCOMING_SPEED)

behavior GuardPedestrianBehavior():
    while True:
        wait

guardPed = new Pedestrian at guardPedSpawnPt,
    with regionContainedIn None,
    with behavior GuardPedestrianBehavior()

param OPT_TERMINATION_DISTANCE = Range(50, 80)

require 15 <= (distance from egoSpawnPt to npcSpawnPt) <= 35
terminate when (distance from egoSpawnPt to ego) > globalParameters.OPT_TERMINATION_DISTANCE