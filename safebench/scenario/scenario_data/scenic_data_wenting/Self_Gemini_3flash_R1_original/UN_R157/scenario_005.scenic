description = "Ego vehicle encounters a stationary motorcycle in the center of a highway curve, requiring emergency braking."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.yamaha.yzf'
param weather = 'ClearNoon'

param motoDist = Range(40, 60)

egoLane = Uniform(*network.lanes)

egoSpawnPt = new OrientedPoint on egoLane.centerline

motoSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.motoDist

param EGO_SPEED = Range(20, 25)
param EGO_BRAKE = 1.0
param SAFE_DIST = 25
EGO_MODEL = 'vehicle.tesla.model3'

behavior EgoBehavior(speed, dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, dist):
        take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.SAFE_DIST)

param ADV_BRAKE = 1.0

behavior StationaryBehavior():
    while True:
        take SetBrakeAction(globalParameters.ADV_BRAKE)

AdvAgent = new Motorcycle at motoSpawnPt,
    with blueprint MODEL,
    with behavior StationaryBehavior()

require 40 <= (distance from egoSpawnPt to motoSpawnPt) <= 60
terminate when (distance from ego to AdvAgent) > 70