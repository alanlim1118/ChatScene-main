description = "Ego drives straight; pedestrian sprints from behind bus stop onto road and stops."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_BLOCKER_DISTANCE = Range(20, 35)
param OPT_GEO_PED_X_OFFSET = Range(-0.5, 0.5)
param OPT_GEO_PED_Y_OFFSET = Range(1.0, 2.5)

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

# Used for placing other actors relative to ego and for pedestrian stop target.
egoInitLane = network.laneAt(egoSpawnPt.position)

busStopAnchorPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_DISTANCE
busStopSpawnPt = new OrientedPoint right of busStopAnchorPt by 4

PED_SHIFT = globalParameters.OPT_GEO_PED_X_OFFSET @ globalParameters.OPT_GEO_PED_Y_OFFSET
pedSpawnPt = new OrientedPoint at busStopSpawnPt offset by PED_SHIFT

param OPT_EGO_SPEED = Range(7, 10)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param OPT_ADV_SPEED = Range(2.5, 4.5)
param OPT_ADV_THRESHOLD = Range(15, 25)
param OPT_STOP_DISTANCE = Range(0.5, 1.5)

behavior PedestrianBehavior(actor_reference, target_lane, adv_speed, threshold, stop_dist):
	do CrossingBehavior(actor_reference, min_speed=adv_speed, threshold=threshold) until (distance from self to target_lane.centerline <= stop_dist)
	take SetWalkingSpeedAction(0)
	while True:
		wait

ped = new Pedestrian at pedSpawnPt,
	facing 90 deg relative to pedSpawnPt.heading,
	with behavior PedestrianBehavior(ego, egoInitLane, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_THRESHOLD, globalParameters.OPT_STOP_DISTANCE)

busStop = new BusStop at busStopSpawnPt,
	with heading busStopSpawnPt.heading,
	with regionContainedIn None

monitor TrafficLights():
	freezeTrafficLights()
	while True:
		if withinDistanceToTrafficLight(ego, 100):
			setClosestTrafficLightStatus(ego, "green")
		wait

require monitor TrafficLights()
require 40 <= (distance from egoSpawnPt to intersection) <= 60
terminate when (distance from ego to egoSpawnPt) > (globalParameters.OPT_GEO_BLOCKER_DISTANCE + 20)