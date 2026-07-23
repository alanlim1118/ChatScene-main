description = "Blue ego vehicle signals left at a T-junction and stops while cross-traffic halts to allow a pedestrian to cross."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))
egoInitLane = Uniform(*filter(lambda l: not any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers), intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoTrajectoryLine = egoManeuver.startLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline
adv1Maneuver = Uniform(*egoManeuver.conflictingManeuvers)
adv1SpawnPt = new OrientedPoint in adv1Maneuver.startLane.centerline
adv1Trajectory = [adv1Maneuver.startLane, adv1Maneuver.connectingLane, adv1Maneuver.endLane]
adv2Maneuver = Uniform(*egoManeuver.conflictingManeuvers)
adv2SpawnPt = new OrientedPoint in adv2Maneuver.startLane.centerline
adv2Trajectory = [adv2Maneuver.startLane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]
pedSpawnPt = new OrientedPoint in intersection.polygon

param EGO_SPEED = Range(7, 10)
param YIELD_DIST = Range(8, 12)
EGO_BRAKE = 1.0

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyPedestrians(self, globalParameters.YIELD_DIST):
		while withinDistanceToAnyPedestrians(self, globalParameters.YIELD_DIST):
			take SetBrakeAction(EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)
param ADV_BRAKE = Range(0.8, 1.0)
param SAFE_DIST = Range(8, 12)

behavior AdvBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)
    interrupt when withinDistanceToAnyPedestrians(self, globalParameters.SAFE_DIST):
        while withinDistanceToAnyPedestrians(self, globalParameters.SAFE_DIST):
            take SetBrakeAction(globalParameters.ADV_BRAKE)

adv = new Car at adv1SpawnPt,
    with blueprint MODEL,
    with heading adv1SpawnPt.heading,
    with behavior AdvBehavior()

behavior Adv2Behavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)
    interrupt when withinDistanceToAnyPedestrians(self, globalParameters.SAFE_DIST):
        while withinDistanceToAnyPedestrians(self, globalParameters.SAFE_DIST):
            take SetBrakeAction(globalParameters.ADV_BRAKE)

adv2 = new Car at adv2SpawnPt,
    with blueprint MODEL,
    with heading adv2SpawnPt.heading,
    with behavior Adv2Behavior()

param PED_MIN_SPEED = Range(0.8, 1.2)
param PED_THRESHOLD = Range(8, 12)

behavior PedestrianBehavior():
	do CrossingBehavior(reference_actor=ego, min_speed=globalParameters.PED_MIN_SPEED, threshold=globalParameters.PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
    with behavior PedestrianBehavior()

require 20 <= (distance from egoSpawnPt to intersection) <= 40
terminate when ((ego in intersection) and not (ped in intersection))