description = "Ego vehicle sharply decelerates to avoid a taxi emerging from the left and cutting across its path."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoInitLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advRefPt = new OrientedPoint ahead of egoSpawnPt by Range(15, 25)
advSpawnPt = new OrientedPoint left of advRefPt by Range(3, 6), facing toward advRefPt

param EGO_SPEED = Range(8, 12)
param EGO_BRAKE = Range(0.8, 1.0)
SAFE_DIST = 20

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, SAFE_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(5, 9)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()