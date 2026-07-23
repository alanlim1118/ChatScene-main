description = "Using map ../../maps/Town05.xodr with carla map Town05 and weather ClearSunset"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuverCands = [m for m in egoInitLane.maneuvers if m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way]
intersection = egoManeuverCands[0].intersection
egoManeuver = Uniform(*[m for m in egoManeuverCands if m.intersection is intersection])

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline

advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

leftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
leftLane = leftManeuver.startLane
pedSpawnPt = new OrientedPoint in leftLane.centerline

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero'

param PED_MIN_SPEED = 1.0
param PED_THRESHOLD = 20

behavior PedestrianBehavior():
    do CrossingBehavior(ego, globalParameters.PED_MIN_SPEED, globalParameters.PED_THRESHOLD)

ped = new Pedestrian right of pedSpawnPt by 3,
    facing ego.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param TERM_DIST = 100

require 20 <= (distance to intersection) <= 30
terminate when (distance from ego to pedSpawnPt) > globalParameters.TERM_DIST