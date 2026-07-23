description = "Ego vehicle follows a front vehicle making a left turn at a nighttime urban intersection, braking for oncoming traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
frontManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
frontInitLane = frontManeuver.startLane
frontSpawnPt = new OrientedPoint following frontInitLane.orientation from frontInitLane.centerline.start for Range(40, 60)
egoSpawnPt = new OrientedPoint following frontInitLane.orientation from frontInitLane.centerline.start for Range(10, 30)
frontTrajectory = [frontInitLane, frontManeuver.connectingLane, frontManeuver.endLane]
egoInitLane = frontInitLane
egoTrajectory = frontTrajectory
oncomingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, frontManeuver.conflictingManeuvers))
oncomingInitLane = oncomingManeuver.startLane
oncomingSpawnPt = new OrientedPoint following oncomingInitLane.orientation from oncomingInitLane.centerline.start for Range(20, 40)
oncomingTrajectory = [oncomingInitLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane]

param OPT_EGO_SPEED = Range(8, 12)
param OPT_BRAKE_DISTANCE = Range(8, 12)

behavior EgoBehavior(speed, brake_dist):
    try:
        do FollowTrajectoryBehavior(target_speed=speed, trajectory=egoTrajectory)
    interrupt when withinDistanceToAnyCars(self, brake_dist):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DISTANCE)

param OPT_FRONT_SPEED = Range(5, 8)
param OPT_YIELD_DISTANCE = Range(10, 15)

behavior LeftTurnYieldBehavior(speed, yield_dist):
    try:
        do FollowTrajectoryBehavior(target_speed=speed, trajectory=frontTrajectory)
    interrupt when withinDistanceToAnyObjs(self, yield_dist):
        take SetBrakeAction(1)

leftTurnAdv = new Car at frontSpawnPt,
    with regionContainedIn None,
    with behavior LeftTurnYieldBehavior(globalParameters.OPT_FRONT_SPEED, globalParameters.OPT_YIELD_DISTANCE)

param OPT_ONCOMING_SPEED = Range(10, 18)

behavior StraightForwardBehavior(speed):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=oncomingTrajectory)

oncomingAdv = new Car at oncomingSpawnPt,
    with regionContainedIn None,
    with behavior StraightForwardBehavior(globalParameters.OPT_ONCOMING_SPEED)

param OPT_STRAIGHT_SPEED = Range(8, 12)

behavior StraightTravelBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

straightAdv = new Car following oncomingInitLane.orientation from oncomingInitLane.centerline.start for Range(5, 15),
    facing oncomingInitLane.orientation,
    with regionContainedIn None,
    with behavior StraightTravelBehavior(globalParameters.OPT_STRAIGHT_SPEED)

require 15 <= (distance from ego to leftTurnAdv) <= 45
terminate when (distance from leftTurnAdv to frontSpawnPt > 20) and (not (ego in intersection)) and (not (leftTurnAdv in intersection)) and (not (oncomingAdv in intersection)) and (not (straightAdv in intersection))