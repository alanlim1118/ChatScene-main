description = "Ego vehicle yields to cross-traffic and turning vehicles at a T-junction before turning right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advStraightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advLeftManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))

advStraightInitLane = advStraightManeuver.startLane
advStraightSpawnPt = new OrientedPoint in advStraightInitLane.centerline

advLeftInitLane = advLeftManeuver.startLane
advLeftSpawnPt = new OrientedPoint in advLeftInitLane.centerline

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advStraightTrajectory = [advStraightInitLane, advStraightManeuver.connectingLane, advStraightManeuver.endLane]
advLeftTrajectory = [advLeftInitLane, advLeftManeuver.connectingLane, advLeftManeuver.endLane]

param EGO_SPEED = Range(7, 10)
EGO_BRAKE = 1.0
param SAFETY_DIST = Range(8, 12)

behavior EgoBehavior(trajectory):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
        while withinDistanceToAnyObjs(self, globalParameters.SAFETY_DIST):
            take SetBrakeAction(EGO_BRAKE)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advStraightSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advStraightTrajectory)

leftAdversary = new Car at advLeftSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advLeftTrajectory)

require (distance from egoSpawnPt to intersection) >= 5
terminate when ego in egoManeuver.endLane