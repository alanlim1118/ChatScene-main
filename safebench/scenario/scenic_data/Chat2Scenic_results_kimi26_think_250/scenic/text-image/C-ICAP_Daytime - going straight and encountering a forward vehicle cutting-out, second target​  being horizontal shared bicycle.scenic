description = "Ego vehicle approaches an intersection behind a blue vehicle where a stationary bicycle lies on the zebra crossing."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoLane = Uniform(*filter(lambda l: l.road.forwardLanes is not None and l.road.backwardLanes is not None and len(l.road.forwardLanes.lanes) == 1 and len(l.road.backwardLanes.lanes) == 1, intersection.incomingLanes))
advSpawnPt = new OrientedPoint in egoLane.centerline
egoSpawnPt = new OrientedPoint behind advSpawnPt by Range(5, 10), facing advSpawnPt.heading
zebraPt = new OrientedPoint on egoLane.centerline[-1], facing advSpawnPt.heading
bicycleSpawnPt = new OrientedPoint right of zebraPt by Range(2, 4), facing toward zebraPt

param OPT_EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

bicycle = new Bicycle at bicycleSpawnPt