description = "Ego vehicle delays lane change to overtake a slow lead vehicle on a shared road due to unsafe boundary conditions, waiting for a safe gap."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoSection = network.laneSectionAt(egoSpawnPt)
egoLane = egoSection.lane
road = egoLane.road

param OPT_DISTANCE_lead = Range(10,20)

param OPT_DIST_bicycle = Range(10,20)

leadSpawnPt = new OrientedPoint following egoSection.orientation from egoSpawnPt for globalParameters.OPT_DISTANCE_lead

bicycleIntPt = new OrientedPoint following egoSection.orientation from leadSpawnPt for globalParameters.OPT_DIST_bicycle
bicycleSpawnPt = new OrientedPoint right of bicycleIntPt by 1.0

oncomingLane = Uniform(*road.backwardLanes.lanes)
oncomingSection = Uniform(*oncomingLane.sections)
oncomingSpawnPt = new OrientedPoint on oncomingSection.centerline

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoSection,
    with blueprint MODEL

param OPT_LEADING_SPEED = Range(4, 7)

leadVehicle = new Car at leadSpawnPt,
    with blueprint MODEL,
    with regionContainedIn egoSection,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED)

param OPT_ONCOMING_SPEED = Range(7, 10)

oncomingCar = new Car at oncomingSpawnPt,
    with regionContainedIn oncomingSection,
    with blueprint MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_ONCOMING_SPEED, is_oppositeTraffic=True)

param OPT_BIKE_SPEED = Range(4, 6)

behavior BicycleBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

bicycle = new Bicycle at bicycleSpawnPt,
    with behavior BicycleBehavior(globalParameters.OPT_BIKE_SPEED),
    with regionContainedIn None

require (distance from egoSpawnPt to oncomingCar) >= 60
require (distance from egoSpawnPt to bicycle) >= 20
terminate when (ego in oncomingSection) and (distance from ego to egoSpawnPt) > 20
