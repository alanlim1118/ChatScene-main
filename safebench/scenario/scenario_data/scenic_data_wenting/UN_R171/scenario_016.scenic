description = "Ego vehicle approaches a significantly slower vehicle on a straight road with a high-speed closing differential (>50 km/h)."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

advInitLane = egoInitLane
advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(30, 50)

advTrajectory = [advInitLane]

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

param OPT_ADV_SPEED = Range(2, 4)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

advCar = new NPCCar at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(advCar, 100):
            setClosestTrafficLightStatus(advCar, "green")
        wait

require monitor TrafficLights()
terminate when (distance from ego to egoSpawnPt) > 150
