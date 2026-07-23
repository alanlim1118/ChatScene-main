description = "Ego vehicle drives straight through a four-way intersection with cross traffic, turning vehicles, and pedestrians."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
leftAdvManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
leftAdvInitLane = leftAdvManeuver.startLane
leftAdvSpawnPt = new OrientedPoint in leftAdvInitLane.centerline
rightAdvManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
rightAdvInitLane = rightAdvManeuver.startLane
rightAdvSpawnPt = new OrientedPoint in rightAdvInitLane.centerline
oppAdvManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
oppAdvInitLane = oppAdvManeuver.startLane
oppAdvSpawnPt = new OrientedPoint in oppAdvInitLane.centerline
pedSpawnPt = new OrientedPoint at egoInitLane.centerline[-1]

param EGO_SPEED = Range(7, 10)
param SAFETY_DIST = Range(10, 15)
BRAKE = 1.0

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        while withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
            take SetBrakeAction(BRAKE)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdvBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new NPCCar at leftAdvSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior([leftAdvInitLane, leftAdvManeuver.connectingLane, leftAdvManeuver.endLane])

param ADV2_SPEED = Range(7, 10)

behavior Adv2Behavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV2_SPEED, trajectory=trajectory)

adversary2 = new NPCCar at rightAdvSpawnPt,
    with blueprint MODEL,
    with behavior Adv2Behavior([rightAdvInitLane, rightAdvManeuver.connectingLane, rightAdvManeuver.endLane])

param ADV3_SPEED = Range(7, 10)

behavior Adv3Behavior():
    turnManeuver = Uniform(*filter(lambda m: m.type is not ManeuverType.STRAIGHT, oppAdvInitLane.maneuvers))
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV3_SPEED, trajectory=[oppAdvInitLane, turnManeuver.connectingLane, turnManeuver.endLane])

adversary3 = new NPCCar at oppAdvSpawnPt,
    with blueprint MODEL,
    with behavior Adv3Behavior()

param ADV4_SPEED = Range(0.8, 1.4)

behavior PedestrianCrossingBehavior():
    do WalkForwardBehavior(speed=globalParameters.ADV4_SPEED)

adversary4 = new Pedestrian right of pedSpawnPt by 2,
    facing 90 deg relative to pedSpawnPt.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossingBehavior()

EGO_INIT_DIST = [20, 25]
ADV_INIT_DIST = [15, 20]

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary3 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to intersection) > (distance from egoSpawnPt to intersection)