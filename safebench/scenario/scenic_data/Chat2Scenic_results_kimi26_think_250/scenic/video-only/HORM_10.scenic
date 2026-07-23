description = "Ego vehicle follows a lead car on an on-ramp and merges into multi-lane highway traffic."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

allSections = [sec for road in network.roads for lane in road.lanes for sec in lane.sections]
egoLaneSec = Uniform(*filter(lambda s: s._laneToRight is None and s._laneToLeft is not None and len(s.lane.maneuvers) > 0, allSections))
egoInitLane = egoLaneSec.lane
leftLaneSec = egoLaneSec._laneToLeft
advInitLane = leftLaneSec.lane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
leadSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(15, 30)
advSpawnPt = new OrientedPoint in leftLaneSec.centerline
egoManeuver = Uniform(*egoInitLane.maneuvers)
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

param EGO_SPEED = Range(8, 12)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

param ADV2_SPEED = Range(7, 10)

behavior Adv2Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV2_SPEED)

adversary2 = new Car at advSpawnPt,
	with behavior Adv2Behavior()

param ADV3_SPEED = Range(7, 10)

behavior Adv3Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV3_SPEED)

adversary3 = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior Adv3Behavior()