description = "Ego vehicle merges from a curved suburban on-ramp onto a multi-lane road behind a red car."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: any(r.forwardLanes is not None and len(r.forwardLanes.lanes) >= 2 for r in i.roads) and any((r.forwardLanes is None or len(r.forwardLanes.lanes) < 2) for r in i.roads), network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.startLane.road != m.endLane.road and (m.startLane.road.forwardLanes is None or len(m.startLane.road.forwardLanes.lanes) < 2) and m.endLane.road.forwardLanes is not None and len(m.endLane.road.forwardLanes.lanes) >= 2 and m.endLane.group is m.endLane.road.forwardLanes and all(sec._laneToRight is None for sec in m.endLane.sections), intersection.maneuvers))

egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

endLane = egoManeuver.endLane
endLaneSec = Uniform(*endLane.sections)
advInitLane = endLaneSec._laneToLeft.lane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advInitLane]

param EGO_SPEED = Range(8, 12)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 40
terminate when ego in endLane