description = "Ego vehicle turning left at a signaled urban intersection encounters a pedestrian in the crosswalk."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.isSignalized, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

targetLane = egoManeuver.endLane
pedSpawnPt = new OrientedPoint at targetLane.centerline[0]

param EGO_SPEED = Range(6, 9)
param SAFETY_DISTANCE = Range(5, 8)

behavior EgoBehavior(trajectory, speed, safety_dist):
	do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory, globalParameters.EGO_SPEED, globalParameters.SAFETY_DISTANCE)

param PED_MIN_SPEED = 1.0
param PED_THRESHOLD = 20

behavior PedestrianBehavior():
	do CrossingBehavior(ego, globalParameters.PED_MIN_SPEED, globalParameters.PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
	facing 90 deg relative to pedSpawnPt.heading,
	with regionContainedIn None,
	with behavior PedestrianBehavior()

monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 50):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightMonitor()
require 15 <= (distance from egoSpawnPt to intersection) <= 25
terminate when (distance from ego to egoSpawnPt) > 40