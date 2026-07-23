description = "Red ego vehicle merges from a curved right side road onto a main vertical roadway as a blue adversarial vehicle travels straight down the main road, creating a potential conflict at the merge point."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

maneuverPairs = []
for inter in network.intersections:
    for advMan in inter.maneuvers:
        if advMan.type is ManeuverType.STRAIGHT:
            advLane = advMan.startLane
            if len(advLane.group.lanes) > 1 and advLane.sections[-1]._laneToLeft is None:
                for egoMan in inter.maneuvers:
                    if egoMan in advMan.conflictingManeuvers and egoMan.type in (ManeuverType.LEFT_TURN, ManeuverType.RIGHT_TURN):
                        maneuverPairs.append((egoMan, advMan))

egoManeuver, advManeuver = Uniform(*maneuverPairs)
egoInitLane = egoManeuver.startLane
advInitLane = advManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint in advInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(8, 10)

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