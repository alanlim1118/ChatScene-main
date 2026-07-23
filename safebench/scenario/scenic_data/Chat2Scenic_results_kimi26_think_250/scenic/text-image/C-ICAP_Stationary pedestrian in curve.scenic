description = "VUT follows VT on a three-lane straight road as VT cuts left, with child and dog obstacles ahead in the right lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_EGO_ADV_DISTANCE = Range(10, 20)
param OPT_GEO_ADV_OBSTACLE_DISTANCE = Range(40, 70)

road = Uniform(*filter(lambda r: r.forwardLanes is not None and len(r.forwardLanes.lanes) >= 3, network.roads))
leftLane = road.forwardLanes.lanes[0]
middleLane = road.forwardLanes.lanes[1]
rightLane = road.forwardLanes.lanes[2]

advSpawnPt = new OrientedPoint on middleLane.centerline
egoSpawnPt = new OrientedPoint behind advSpawnPt by globalParameters.OPT_GEO_EGO_ADV_DISTANCE

pedRefPt = new OrientedPoint ahead of advSpawnPt by globalParameters.OPT_GEO_ADV_OBSTACLE_DISTANCE
propRefPt = new OrientedPoint ahead of pedRefPt by 2
pedSpawnPt = new OrientedPoint right of pedRefPt by 3.6
propSpawnPt = new OrientedPoint right of propRefPt by 3.6

param OPT_EGO_SPEED = Range(5, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(10, 15)

behavior AdvBehavior():
    do LaneChangeBehavior(leftLane.sections[0], target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with behavior AdvBehavior()

behavior HalfSquatBehavior():
    while True:
        wait

halfSquatPed = new Pedestrian at pedSpawnPt,
    facing pedSpawnPt.heading,
    with behavior HalfSquatBehavior()

prop = new Debris at propSpawnPt