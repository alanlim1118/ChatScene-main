description = "Ego vehicle merges onto a highway from an on-ramp."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

rampManeuvers = []
for l in network.lanes:
    for m in l.maneuvers:
        if m.endLane.road != l.road and m.connectingLane is not None:
            rampManeuvers.append(m)

egoManeuver = Uniform(*rampManeuvers)
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint on egoInitLane.centerline

highwayLane = egoManeuver.endLane
mergeRef = highwayLane.centerline.project(egoSpawnPt.position)

advSpawnPt1 = new OrientedPoint behind mergeRef by Range(30, 50)
advSpawnPt2 = new OrientedPoint ahead of mergeRef by Range(10, 30)

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, highwayLane]
advTrajectory1 = [highwayLane]
advTrajectory2 = [highwayLane]

param OPT_EGO_SPEED = Range(15, 20)
param OPT_EGO_SAFETY_DISTANCE = Range(10, 15)

behavior EgoBehavior(speed, trajectory, safety_dist):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoInitLane,
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        egoTrajectory,
        globalParameters.OPT_EGO_SAFETY_DISTANCE
    )

param OPT_NPC_SPEED = Range(15, 25)
param OPT_NPC_SAFETY_DISTANCE = Range(10, 15)

behavior NPCBehavior(speed, trajectory, safety_dist):
    try:
        do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, safety_dist):
        take SetBrakeAction(1.0)

adv1 = new NPCCar at advSpawnPt1,
    with behavior NPCBehavior(
        globalParameters.OPT_NPC_SPEED,
        advTrajectory1,
        globalParameters.OPT_NPC_SAFETY_DISTANCE
    )

adv2 = new NPCCar at advSpawnPt2,
    with behavior NPCBehavior(
        globalParameters.OPT_NPC_SPEED,
        advTrajectory2,
        globalParameters.OPT_NPC_SAFETY_DISTANCE
    )

TERM_DIST = 100

terminate when (distance to egoSpawnPt) > TERM_DIST