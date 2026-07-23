description = "Ego vehicle at an intersection passes an adversarial object on the left turning left into the cross street."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoAdvLaneSecPairs = []
for lane in network.lanes:
    if lane.maneuvers and any(m.type is ManeuverType.LEFT_TURN for m in lane.maneuvers):
        lastSec = lane.sections[-1]
        if lastSec._laneToLeft is not None:
            egoAdvLaneSecPairs.append((lastSec, lastSec._laneToLeft))

egoLaneSec, advLaneSec = Uniform(*egoAdvLaneSecPairs)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoLaneSec.lane.maneuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in advLaneSec.centerline

param EGO_SPEED = Range(3, 5)

behavior EgoBehavior():
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(5, 8)

behavior AdversaryBehavior():
	advManeuver = [m for m in advLaneSec.lane.maneuvers if m.type is ManeuverType.LEFT_TURN][0]
	advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=advTrajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()