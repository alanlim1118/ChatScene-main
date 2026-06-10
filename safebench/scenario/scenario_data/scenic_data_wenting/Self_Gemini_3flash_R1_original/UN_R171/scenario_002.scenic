description = "Ego vehicle delays lane change to overtake a slow lead vehicle on a shared road due to unsafe boundary conditions, waiting for a safe gap."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

road = Uniform(*filter(lambda r: r.forwardLanes is not None and r.backwardLanes is not None, network.roads))
egoLane = Uniform(*road.forwardLanes.lanes)
egoSection = Uniform(*egoLane.sections)
egoSpawnPt = new OrientedPoint on egoSection.centerline

leadSpawnPt = new OrientedPoint following egoSection.orientation from egoSpawnPt for Range(10, 20)

bicycleIntPt = new OrientedPoint following egoSection.orientation from leadSpawnPt for Range(10, 20)
bicycleSpawnPt = new OrientedPoint right of bicycleIntPt by 1.0

oncomingLane = Uniform(*road.backwardLanes.lanes)
oncomingSection = Uniform(*oncomingLane.sections)
oncomingSpawnPt = new OrientedPoint on oncomingSection.centerline

egoTrajectory = [egoLane]

param OPT_EGO_SPEED = Range(7, 10)
param OPT_OVERTAKE_DIST = Range(12, 15)

behavior EgoBehavior(speed, overtake_dist, target_lane_sec):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, overtake_dist)
    # The ego vehicle delays the maneuver to wait for a safe gap in oncoming traffic
    wait for Range(3, 5) seconds
    # The ego vehicle completes the lane change to the oncoming lane to overtake
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, is_oppositeTraffic=True, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed, is_oppositeTraffic=True)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoSection,
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_OVERTAKE_DIST,
        oncomingSection
    )

param OPT_LEADING_SPEED = globalParameters.OPT_EGO_SPEED - 3

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