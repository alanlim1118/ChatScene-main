description = "Ego vehicle approaches a significantly slower vehicle on a straight road with a high-speed closing differential (>50 km/h)."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoInitLane = Uniform(*filter(lambda l: l.centerline.length > 100, network.lanes))
egoSpawnPt = new OrientedPoint on egoInitLane.centerline

advInitLane = egoInitLane
advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(30, 50)

egoTrajectory = [egoInitLane]
advTrajectory = [advInitLane]

param EGO_SPEED = Range(18, 22)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(2, 4)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

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