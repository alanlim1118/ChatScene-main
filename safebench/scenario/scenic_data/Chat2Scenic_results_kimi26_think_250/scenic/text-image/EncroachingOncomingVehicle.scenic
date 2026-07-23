description = "Oncoming vehicle encroaches across the solid yellow divider into the subject vehicle's lane, creating a head-on collision scenario."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

twoWayRoads = filter(lambda r: r.forwardLanes is not None and r.backwardLanes is not None, network.roads)
road = Uniform(*twoWayRoads)
egoLane = Uniform(*road.forwardLanes.lanes)
advLane = Uniform(*road.backwardLanes.lanes)
egoSpawnPt = new OrientedPoint in egoLane.centerline
advSpawnPt = new OrientedPoint in advLane.centerline

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_THROTTLE = 0.5
param ADV_STEER = -0.3

behavior AdversaryBehavior():
    past = globalParameters.ADV_STEER
    while True:
        take RegulatedControlAction(throttle=globalParameters.ADV_THROTTLE, steer=globalParameters.ADV_STEER, past_steer=past)
        past = globalParameters.ADV_STEER

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()