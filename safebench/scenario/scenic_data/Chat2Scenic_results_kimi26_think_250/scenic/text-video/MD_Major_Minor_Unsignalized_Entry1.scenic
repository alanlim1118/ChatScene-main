description = "Ego vehicle travels straight through a T-junction trailed by an adversary, while a second adversary turns left from the right intersection arm."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

adv1SpawnPt = new OrientedPoint behind egoSpawnPt by Range(10, 20)

adv2Maneuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT, egoManeuver.conflictingManeuvers))
adv2InitLane = adv2Maneuver.startLane
adv2SpawnPt = new OrientedPoint in adv2InitLane.centerline

egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
adv2Trajectory = [adv2Maneuver.startLane, adv2Maneuver.connectingLane, adv2Maneuver.endLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adv1 = new Car at adv1SpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(egoTrajectory)

adv2 = new Car at adv2SpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(adv2Trajectory)

require 10 <= (distance from egoSpawnPt to adv1SpawnPt) <= 20
terminate when ego in egoManeuver.endLane