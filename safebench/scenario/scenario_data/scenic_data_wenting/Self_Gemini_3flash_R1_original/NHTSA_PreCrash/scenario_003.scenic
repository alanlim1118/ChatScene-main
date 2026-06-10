description = "Ego vehicle runs a red light at an intersection, colliding with a lateral vehicle, in an urban area with a 35 mph speed limit under clear daylight."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way and i.isSignalized, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToObjsInLane(self, 10):
		take SetBrakeAction(1.0)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "red")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

require monitor TrafficLightMonitor()
require 15 <= (distance from egoSpawnPt to intersection) <= 25
require 15 <= (distance from advSpawnPt to intersection) <= 25

terminate when ego intersects adversary