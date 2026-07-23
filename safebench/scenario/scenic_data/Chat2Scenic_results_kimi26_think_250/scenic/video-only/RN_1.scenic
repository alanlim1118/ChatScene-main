description = "Ego vehicle navigates a roundabout in the outer lane alongside a red adversary to execute a leftward turn."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*network.intersections)

egoInitLane = Uniform(*filter(lambda lane: all([sec._laneToLeft is not None for sec in lane.sections]), intersection.incomingLanes))
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advInitLane = egoInitLane.sectionAt(egoSpawnPt).laneToLeft.lane
advSpawnPt = new OrientedPoint in advInitLane.centerline

egoManeuver = Uniform(*egoInitLane.maneuvers)
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*advInitLane.maneuvers)
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(5, 8)

behavior AdversaryBehavior():
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

adversary = new Car at advSpawnPt,
    with color Color(1, 0, 0),
    with behavior AdversaryBehavior()

require 15 <= (distance from egoSpawnPt to intersection) <= 50
require 15 <= (distance from advSpawnPt to intersection) <= 50
terminate when (ego in egoManeuver.endLane) and ((distance from ego to egoSpawnPt) > 15)