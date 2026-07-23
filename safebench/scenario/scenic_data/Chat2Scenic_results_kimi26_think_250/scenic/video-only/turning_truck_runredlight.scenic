description = "Ego vehicle preparing to turn left at an intersection is struck by a dump truck running a red light, while a crane truck is visible driving away on the cross street."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

truck2InitLane = Uniform(*egoManeuver.endLane.group.lanes)
truck2SpawnPt = new OrientedPoint in truck2InitLane.centerline

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_TRUCK_SPEED = Range(12, 18)

advTruck = new Truck at advSpawnPt,
    facing advSpawnPt.heading,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.ADV_TRUCK_SPEED, trajectory=advTrajectory)

param TRUCK_AWAY_SPEED = Range(8, 12)

craneTruck = new Truck at truck2SpawnPt,
    facing truck2SpawnPt.heading,
    with behavior FollowLaneBehavior(target_speed=globalParameters.TRUCK_AWAY_SPEED)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(advTruck, 100):
            setClosestTrafficLightStatus(advTruck, "red")
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 30 <= (distance to intersection) <= 50
terminate when (advTruck in advManeuver.endLane) and (ego can see craneTruck)