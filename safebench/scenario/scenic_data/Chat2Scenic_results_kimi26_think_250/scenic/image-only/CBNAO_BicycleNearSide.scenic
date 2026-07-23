description = "Vehicle under test collides with a nearside bicyclist crossing from behind parked vehicles without braking."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_SPEED = Range(5, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param BICYCLE_MIN_SPEED = 1.5
param BICYCLE_THRESHOLD = 18

behavior BicycleBehavior():
    do CrossingBehavior(ego, globalParameters.BICYCLE_MIN_SPEED, globalParameters.BICYCLE_THRESHOLD)

bicycle = new Bicycle right of egoSpawnPt by 3.5,
    facing 90 deg relative to egoSpawnPt.heading,
    with behavior BicycleBehavior(),
    with regionContainedIn None

stationary = new Car behind egoSpawnPt by 10,
    facing egoSpawnPt.heading,
    with regionContainedIn None

parkedCar = new Car behind egoSpawnPt by 20,
    facing egoSpawnPt.heading,
    with regionContainedIn None

require 3 <= (distance from ego to bicycle) <= 6
terminate when ego intersects bicycle