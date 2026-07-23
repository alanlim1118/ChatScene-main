description = "Ego vehicle waits in a four-way intersection to yield to cross-traffic and pedestrians before proceeding straight."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoSpawnPt = new OrientedPoint in egoManeuver.connectingLane.centerline
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

advNorthSpawnPt = new OrientedPoint in intersection.incomingLanes[0].centerline
advSouthSpawnPt = new OrientedPoint in intersection.incomingLanes[1].centerline
advEastSpawnPt = new OrientedPoint in intersection.incomingLanes[2].centerline

corners = list(intersection.polygon.exterior.coords)[:-1]
pedSpawnPt1 = new OrientedPoint at corners[0]
pedSpawnPt2 = new OrientedPoint at corners[1]
pedSpawnPt3 = new OrientedPoint at corners[2]
pedSpawnPt4 = new OrientedPoint at corners[3]

param EGO_SPEED = Range(5, 10)
param EGO_WAIT_DIST = Range(5, 10)

behavior EgoBehavior(speed, wait_dist):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=egoTrajectory) until (self in intersection and (withinDistanceToAnyCars(self, wait_dist) or withinDistanceToAnyPedestrians(self, wait_dist)))
    while (self in intersection and (withinDistanceToAnyCars(self, wait_dist) or withinDistanceToAnyPedestrians(self, wait_dist))):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        wait
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.EGO_WAIT_DIST)

param ADV_NORTH_SPEED = Range(10, 20)

behavior AdvNorthBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

advCar = new Car at advNorthSpawnPt,
    facing advNorthSpawnPt.heading,
    with behavior AdvNorthBehavior(globalParameters.ADV_NORTH_SPEED)

param ADV_SOUTH_SPEED = Range(10, 20)

behavior AdvSouthBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

advSouthCar = new Car at advSouthSpawnPt,
    facing advSouthSpawnPt.heading,
    with behavior AdvSouthBehavior(globalParameters.ADV_SOUTH_SPEED)

param ADV_EAST_SPEED = Range(10, 20)

behavior AdvEastBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

advEastCar = new Car at advEastSpawnPt,
    facing advEastSpawnPt.heading,
    with behavior AdvEastBehavior(globalParameters.ADV_EAST_SPEED)

param PED_SPEED = Range(1, 3)

behavior PedestrianWalkPathBehavior(speed):
    do WalkForwardBehavior(speed)

advPed = new Pedestrian at pedSpawnPt1,
    facing toward pedSpawnPt2,
    with regionContainedIn None,
    with behavior PedestrianWalkPathBehavior(globalParameters.PED_SPEED)

terminate when (distance from ego to intersection > 15)