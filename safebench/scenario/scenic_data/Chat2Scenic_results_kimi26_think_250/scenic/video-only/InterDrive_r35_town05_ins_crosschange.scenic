description = "Ego vehicle approaching intersection for right turn is blocked by two adversaries in current and adjacent lanes."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*filter(lambda lane: all([sec._laneToLeft is None and sec._laneToRight is not None for sec in lane.sections]), intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

adv1SpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(10, 20)

adv1LaneSec = egoInitLane.sectionAt(adv1SpawnPt)
adv2InitLane = adv1LaneSec.laneToRight.lane
adv2SpawnPt = new OrientedPoint in adv2InitLane.centerline

param OPT_EGO_SPEED = Range(5, 8)
param OPT_YIELD_DIST = Range(10, 15)

behavior EgoBehavior():
    while True:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED) until (withinDistanceToAnyCars(self, globalParameters.OPT_YIELD_DIST))
        if not withinDistanceToAnyCars(self, globalParameters.OPT_YIELD_DIST):
            terminate
        while withinDistanceToAnyCars(self, globalParameters.OPT_YIELD_DIST):
            take SetThrottleAction(0)
            take SetBrakeAction(1)
            wait

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior EgoBehavior()

adversary = new Car at adv1SpawnPt,
    with blueprint MODEL

param ADV2_SPEED = Range(5, 8)

behavior Adv2Behavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

adversary2 = new Car at adv2SpawnPt,
    with blueprint MODEL,
    with behavior Adv2Behavior()