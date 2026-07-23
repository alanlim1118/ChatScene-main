description = "Ego vehicle yields to oncoming traffic while executing a left turn at a four-way intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

param EGO_SPEED = Range(5, 7)
param YIELD_DISTANCE = Range(10, 15)

behavior EgoBehavior(target_speed, yield_distance):
    try:
        do FollowTrajectoryBehavior(target_speed=target_speed, trajectory=egoTrajectory)
    interrupt when withinDistanceToAnyCars(self, yield_distance):
        take SetBrakeAction(1)
        wait until not withinDistanceToAnyCars(self, yield_distance)

ego = new Car at egoSpawnPt,
    facing egoDir,
    with blueprint MODEL,
    with behavior EgoBehavior(EGO_SPEED, YIELD_DISTANCE)

param ADV_SPEED = Range(8, 12)

behavior StraightAdvBehavior():
    do FollowTrajectoryBehavior(target_speed=ADV_SPEED, trajectory=advTrajectory)

adv = new Car at advSpawnPt,
    facing advDir,
    with behavior StraightAdvBehavior()

behavior StraightCarBehavior():
	do FollowTrajectoryBehavior(target_speed=ADV_SPEED, trajectory=advTrajectory)

adversary = new Car at advSpawnPt,
	facing advDir,
	with behavior StraightCarBehavior()

terminate when ego in egoManeuver.endLane