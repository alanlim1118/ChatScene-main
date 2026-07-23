description = "Ego vehicle merges from a curved suburban on-ramp onto a multi-lane road behind a red car."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.intersection is not None and m.startLane.road != m.endLane.road and (m.startLane.road.forwardLanes is None or len(m.startLane.road.forwardLanes.lanes) < 2) and m.endLane.road.forwardLanes is not None and len(m.endLane.road.forwardLanes.lanes) >= 2 and m.endLane.group is m.endLane.road.forwardLanes and all(sec._laneToRight is None for sec in m.endLane.sections), egoInitLane.maneuvers))

endLane = egoManeuver.endLane
endLaneSec = Uniform(*endLane.sections)
advInitLane = endLaneSec._laneToLeft.lane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advInitLane]

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 40
terminate when ego in endLane