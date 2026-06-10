description = "Ego vehicle merges onto a highway from an on-ramp."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.endLane.road != egoInitLane.road and m.connectingLane is not None, egoInitLane.maneuvers))

highwayLane = egoManeuver.endLane
mergeRef = highwayLane.centerline.project(egoSpawnPt.position)

advSpawnPt1 = new OrientedPoint behind mergeRef by Range(30, 50)
advSpawnPt2 = new OrientedPoint ahead of mergeRef by Range(10, 30)

advTrajectory1 = [highwayLane]
advTrajectory2 = [highwayLane]

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoInitLane,
    with blueprint MODEL

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
