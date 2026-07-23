description = "Ego vehicle follows a car merging from a left junction on a wet city road before being overtaken by a sedan."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetCloudyNoon'

intersection = Uniform(*network.intersections)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and all(sec._laneToRight is not None for sec in m.startLane.sections), intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

egoLaneSec = egoInitLane.sectionAt(egoSpawnPt)
rightLaneSec = egoLaneSec._laneToRight
overtakeSpawnPt = new OrientedPoint in rightLaneSec.centerline

param EGO_SPEED = Range(8, 12)
param EGO_BRAKE = Range(0.5, 1.0)
SAFE_DIST = 20

behavior EgoBehavior():
	try:
		do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
	interrupt when withinDistanceToObjsInLane(self, SAFE_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(5, 8)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)
	do AccelerateForwardBehavior()

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param OVERTAKE_SPEED = globalParameters.EGO_SPEED * Uniform(1.3, 1.6)

behavior OvertakeBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OVERTAKE_SPEED)

sedan = new Car at overtakeSpawnPt,
	with blueprint MODEL,
	with behavior OvertakeBehavior()

require 30 <= (distance from egoSpawnPt to intersection) <= 60
require 5 <= (distance from advSpawnPt to intersection) <= 15
terminate when (distance from ego to sedan) > 40 and (distance from ego to egoSpawnPt) > 10