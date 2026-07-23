description = "Ego vehicle collides with a motorcyclist crossing perpendicularly at a four-way intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and len(m.startLane.adjacentLanes) > 0 and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

truckInitLane = Uniform(*egoInitLane.adjacentLanes)
truckSpawnPt = new OrientedPoint in truckInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Truck at advSpawnPt,
	with behavior AdversaryBehavior(advTrajectory)

param OPT_MOTO_SPEED = Range(5, 12)

behavior MotorcycleBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_MOTO_SPEED, trajectory=advTrajectory)

motorcycle = new Motorcycle ahead of advSpawnPt by Range(10, 20),
    facing advSpawnPt.heading,
    with regionContainedIn None,
    with behavior MotorcycleBehavior()
