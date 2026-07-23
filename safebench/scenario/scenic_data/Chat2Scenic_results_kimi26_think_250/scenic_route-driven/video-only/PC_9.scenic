description = "Ego vehicle proceeds straight through a foggy four-way intersection with cross traffic and a pedestrian."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

adv1Maneuver = Uniform(*egoManeuver.conflictingManeuvers)
adv1InitLane = adv1Maneuver.startLane
adv1SpawnPt = new OrientedPoint in adv1InitLane.centerline
adv1Trajectory = [adv1Maneuver.startLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]

adv2Lane = Uniform(*egoInitLane.road.backwardLanes.lanes)
adv2SpawnPt = new OrientedPoint in adv2Lane.centerline

pedRefPt = new OrientedPoint at egoInitLane.centerline.end
pedSpawnPt = new OrientedPoint right of pedRefPt by Range(2, 4)

param OPT_EGO_SPEED = Range(7, 10)

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param ADV_SPEED = globalParameters.OPT_EGO_SPEED

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at adv1SpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(adv1Trajectory)

param OPT_ADV2_SPEED = Range(7, 10)

behavior Adv2Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV2_SPEED)

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