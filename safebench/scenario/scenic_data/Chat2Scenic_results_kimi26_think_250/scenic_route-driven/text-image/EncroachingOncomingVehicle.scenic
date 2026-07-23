description = "Oncoming vehicle encroaches across the solid yellow divider into the subject vehicle's lane, creating a head-on collision scenario."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)
road = egoLane.road
advLane = Uniform(*road.backwardLanes.lanes)
advSpawnPt = new OrientedPoint in advLane.centerline

ego = new Car at egoSpawnPt,
    with blueprint MODEL

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