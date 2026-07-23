description = "Ego vehicle proceeds straight through a foggy four-way intersection with cross traffic and a pedestrian."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

adv1Maneuver = Uniform(*egoManeuver.conflictingManeuvers)
adv1InitLane = adv1Maneuver.startLane
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline
adv1Trajectory = [adv1Maneuver.startLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]

adv2Lane = Uniform(*egoInitLane.road.backwardLanes.lanes)
adv2SpawnPt = new OrientedPoint in adv2Lane.centerline

pedRefPt = new OrientedPoint at egoInitLane.centerline.end
pedSpawnPt = new OrientedPoint right of pedRefPt by Range(2, 4)

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = globalParameters.EGO_SPEED

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at adv1SpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(adv1Trajectory)

param ADV2_SPEED = Range(7, 10)

behavior Adv2Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

adversary2 = new Car at adv2SpawnPt,
	with blueprint MODEL,
	with behavior Adv2Behavior()

behavior StandBehavior():
    while True:
        wait

ped = new Pedestrian at pedSpawnPt,
    with regionContainedIn None,
    with behavior StandBehavior()

EGO_INIT_DIST = [20, 25]
ADV_INIT_DIST = [15, 20]
TERM_DIST = 100

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary2 to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST