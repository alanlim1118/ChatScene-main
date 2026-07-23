description = "Green VUT left turn at T-junction intersecting with VRUs at zebra crossing amid stationary traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and m.intersection is not None and m.intersection.is3Way, egoInitLane.maneuvers))

blocker1SpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(15, 25)
blocker2SpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(30, 40)

egoLaneEndPt = egoInitLane.centerline[-1]
crosswalkRefPt = new OrientedPoint following egoInitLane.orientation from egoLaneEndPt for Range(2, 5)

pedSpawnPt = new OrientedPoint left of crosswalkRefPt by Range(1, 4)
bikeSpawnPt = new OrientedPoint right of crosswalkRefPt by Range(1, 4)
dogSpawnPt = new OrientedPoint left of crosswalkRefPt by Range(4, 7)

ego = new Car at egoSpawnPt,
    with blueprint MODEL

behavior StationaryBehavior():
	while True:
		wait

stationary_car = new Car at blocker1SpawnPt,
	with blueprint MODEL,
	with behavior StationaryBehavior()

behavior StationaryCarBehavior():
	while True:
		wait

stationary_car2 = new Car at blocker2SpawnPt,
	with blueprint MODEL,
	with behavior StationaryCarBehavior()

param BICYCLE_MIN_SPEED = 1.5
param BICYCLE_THRESHOLD = 18

behavior BicycleCrossingBehavior():
    do CrossingBehavior(ego, globalParameters.BICYCLE_MIN_SPEED, globalParameters.BICYCLE_THRESHOLD)

bicycle = new Bicycle at bikeSpawnPt,
    facing 90 deg relative to bikeSpawnPt.heading,
    with behavior BicycleCrossingBehavior(),
    with regionContainedIn None

param DOG_MIN_SPEED = 1.5
param DOG_THRESHOLD = 20

behavior DogCrossingBehavior():
    do CrossingBehavior(ego, globalParameters.DOG_MIN_SPEED, globalParameters.DOG_THRESHOLD)

dog = new Pedestrian at dogSpawnPt,
    facing 90 deg relative to dogSpawnPt.heading,
    with regionContainedIn None,
    with behavior DogCrossingBehavior()

param PED_MIN_SPEED = 1.0
param PED_THRESHOLD = 20

behavior PedestrianCrossingBehavior():
    do CrossingBehavior(ego, globalParameters.PED_MIN_SPEED, globalParameters.PED_THRESHOLD)

pedestrian = new Pedestrian at pedSpawnPt,
    facing 90 deg relative to pedSpawnPt.heading,
    with regionContainedIn None,
    with behavior PedestrianCrossingBehavior()

monitor TrafficLightMonitor():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightMonitor()
require 15 <= (distance from egoSpawnPt to blocker1SpawnPt) <= 25
require 30 <= (distance from egoSpawnPt to blocker2SpawnPt) <= 40
require (distance from egoSpawnPt to pedSpawnPt) >= 5
require (distance from egoSpawnPt to bikeSpawnPt) >= 5
require (distance from egoSpawnPt to dogSpawnPt) >= 5
terminate when (ego in egoManeuver.endLane) or (distance to crosswalkRefPt < 3)