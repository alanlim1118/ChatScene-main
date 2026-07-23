description = "Ego vehicle turns left at an urban intersection, taking evasive action to avoid an obstacle."
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: any(m.type is ManeuverType.LEFT_TURN for m in i.maneuvers), network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
propSpawnPt = new OrientedPoint in egoManeuver.connectingLane.centerline

param OPT_EGO_SPEED = Range(6, 8)
param OPT_EVASIVE_DIST = 12
param OPT_BRAKE_FORCE = 1.0

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_EGO_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_EVASIVE_DIST):
		take SetBrakeAction(globalParameters.OPT_BRAKE_FORCE)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

prop = new Trash at propSpawnPt

TERM_DIST = 50

monitor TrafficLightControl():
	freezeTrafficLights()
	while True:
		if withinDistanceToTrafficLight(ego, 100):
			setClosestTrafficLightStatus(ego, "green")
		wait

require monitor TrafficLightControl()
terminate when (distance from ego to egoSpawnPt) > TERM_DIST