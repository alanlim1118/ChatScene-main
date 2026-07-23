description = "Ego vehicle preparing to turn left at an intersection is struck by a dump truck running a red light, while a crane truck is visible driving away on the cross street."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

truck2InitLane = Uniform(*egoManeuver.endLane.group.lanes)
truck2SpawnPt = new OrientedPoint in truck2InitLane.centerline

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_TRUCK_SPEED = Range(12, 18)

advTruck = new Truck at advSpawnPt,
    facing advSpawnPt.heading,
    with behavior FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_TRUCK_SPEED, trajectory=advTrajectory)

param OPT_TRUCK_AWAY_SPEED = Range(8, 12)

craneTruck = new Truck at truck2SpawnPt,
    facing truck2SpawnPt.heading,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_AWAY_SPEED)

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