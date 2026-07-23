description = "Ego vehicle rear-ends a white SUV that merges from the right and stops suddenly on a multi-lane road, after which the driver exits to inspect the damage."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToRight is not None:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

advLaneSec = egoLaneSec._laneToRight
advSpawnPt = new OrientedPoint in advLaneSec.centerline

advStopPt = new OrientedPoint following roadDirection from egoSpawnPt for Range(15, 25)
pedSpawnPt = new OrientedPoint behind advStopPt by Range(1, 3)

param OPT_EGO_SPEED = Range(8, 12)
param OPT_BRAKE_DIST = Range(5, 10)
param OPT_LANE_CHANGE_DIST = Range(10, 15)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to advStopPt < globalParameters.OPT_LANE_CHANGE_DIST)
        do LaneChangeBehavior(laneSectionToSwitch=advLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DIST)):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED - 1
param OPT_ADV_BRAKE_DIST = Range(3, 6)

behavior AdvBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to advStopPt < globalParameters.OPT_ADV_BRAKE_DIST)
    while True:
        take SetBrakeAction(1)

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn advLaneSec,
    with behavior AdvBehavior()

param OPT_PED_SPEED = Range(0.8, 1.2)
param OPT_INSPECT_TIME = Range(2, 4)

behavior PedestrianInspectBehavior():
    take SetWalkingDirectionAction(advStopPt.heading + 180 deg)
    do WalkForwardBehavior(globalParameters.OPT_PED_SPEED) for 2 seconds
    take SetWalkingSpeedAction(0)
    wait for globalParameters.OPT_INSPECT_TIME seconds
    take SetWalkingDirectionAction(advStopPt.heading)
    do WalkForwardBehavior(globalParameters.OPT_PED_SPEED) for 2 seconds
    take SetWalkingSpeedAction(0)
    wait

pedestrian = new Pedestrian at pedSpawnPt,
    facing advStopPt.heading,
    with regionContainedIn None,
    with behavior PedestrianInspectBehavior()

require (distance from ego to adversary) <= 25
terminate when (ego intersects adversary) and ((distance from pedestrian to pedSpawnPt) < 1.5)