description = "Ego vehicle travels straight through an urban four-way intersection navigating past multiple crossing vehicles and pedestrians."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

oppManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
oppLane = oppManeuver.startLane
oppSpawnPt = new OrientedPoint in oppLane.centerline

crossManeuver1 = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
crossLane1 = crossManeuver1.startLane
crossSpawnPt1 = new OrientedPoint in crossLane1.centerline

crossManeuver2 = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
crossLane2 = crossManeuver2.startLane
crossSpawnPt2 = new OrientedPoint in crossLane2.centerline

advSpawnPt = new OrientedPoint in oppLane.centerline

pedSpawnPt1 = egoInitLane.centerline[-1]
pedSpawnPt2 = oppLane.centerline[-1]
pedSpawnPt3 = new OrientedPoint at oppLane.centerline[-1]

param OPT_EGO_SPEED = Range(5, 8)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    leftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT, oppLane.maneuvers))
    advTrajectory = [leftManeuver.startLane, leftManeuver.connectingLane, leftManeuver.endLane]
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

param STRAIGHT_SPEED = Range(7, 10)

behavior StraightCarBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.STRAIGHT_SPEED)

straightCar = new Car at oppSpawnPt,
    with heading oppSpawnPt.heading,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior StraightCarBehavior()

param CROSS_ADV_SPEED = Range(7, 10)

behavior CrossStraightBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.CROSS_ADV_SPEED)

crossStraightCar = new Car at crossSpawnPt1,
    with heading crossSpawnPt1.heading,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior CrossStraightBehavior()

param CROSS2_ADV_SPEED = Range(7, 10)

behavior CrossStraightBehavior2():
    do FollowLaneBehavior(target_speed=globalParameters.CROSS2_ADV_SPEED)

crossStraightCar2 = new Car at crossSpawnPt2,
    with heading crossSpawnPt2.heading,
    with regionContainedIn None,
    with blueprint MODEL,
    with behavior CrossStraightBehavior2()

PED_MIN_SPEED = 1.0
PED_THRESHOLD = 20

behavior PedestrianBehavior():
    do CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

ped = new Pedestrian right of pedSpawnPt1 by 3,
    facing ego.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()

ped2 = new Pedestrian right of pedSpawnPt2 by 3,
    facing ego.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()

ped3 = new Pedestrian right of pedSpawnPt3 by 3,
    facing ego.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()