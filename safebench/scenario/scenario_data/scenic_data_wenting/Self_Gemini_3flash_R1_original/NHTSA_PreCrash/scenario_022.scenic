description = "Ego vehicle approaches a stopped lead vehicle at an urban intersection with a 35 mph speed limit."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: any(m.type is ManeuverType.STRAIGHT for m in i.maneuvers), network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
advSpawnPt = new OrientedPoint on egoInitLane.centerline
egoSpawnPt = new OrientedPoint behind advSpawnPt by Range(5, 10)

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = egoTrajectory

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 50):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 5 <= (distance from egoSpawnPt to advSpawnPt) <= 10
terminate when (distance from ego to adversary) < 2.5