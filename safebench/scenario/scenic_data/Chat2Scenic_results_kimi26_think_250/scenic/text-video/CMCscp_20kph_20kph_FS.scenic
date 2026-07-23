description = "Ego vehicle collides with a motorcyclist crossing perpendicularly at a four-way intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and len(m.startLane.adjacentLanes) > 0, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

truckInitLane = Uniform(*egoInitLane.adjacentLanes)
truckSpawnPt = new OrientedPoint in truckInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(3, 8)

behavior EgoBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Truck at advSpawnPt,
	with behavior AdversaryBehavior(advTrajectory)

param MOTO_SPEED = Range(5, 12)

behavior MotorcycleBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.MOTO_SPEED, trajectory=advTrajectory)

motorcycle = new Motorcycle ahead of advSpawnPt by Range(10, 20),
    facing advSpawnPt.heading,
    with regionContainedIn None,
    with behavior MotorcycleBehavior()