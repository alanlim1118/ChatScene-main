description = "Ego vehicle approaches sequentially stopped vehicles in its lane and performs a safe stop."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param DIST_EGO_TO_MOTO = Range(30, 40)
param DIST_MOTO_TO_CAR = Range(5, 10)

egoLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoLane.centerline

motoSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.DIST_EGO_TO_MOTO
carSpawnPt = new OrientedPoint following roadDirection from motoSpawnPt for globalParameters.DIST_MOTO_TO_CAR

param EGO_SPEED = Range(6, 10)
param SAFETY_DISTANCE = Range(10, 15)

behavior WaitBehavior():
    while True:
        wait

behavior EgoBehavior(speed, safety_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, safety_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior()

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.SAFETY_DISTANCE)

moto = new Motorcycle at motoSpawnPt,
    with blueprint MODEL,
    with behavior WaitBehavior()

car = new Car at carSpawnPt,
    with behavior WaitBehavior()


require 30 <= (distance from egoSpawnPt to motoSpawnPt) <= 40