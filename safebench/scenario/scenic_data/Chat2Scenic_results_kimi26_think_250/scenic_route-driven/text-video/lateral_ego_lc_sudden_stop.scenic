description = "Ego vehicle attempts a left lane change but rear-ends the lead vehicle after traffic slows and it brakes suddenly."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEAD_DIST = Range(10, 20)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

leadCarSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DIST

leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

leftProjectPt = leftLaneSec.centerline.project(egoSpawnPt.position)
leftCarSpawnPt = new OrientedPoint at leftProjectPt, facing egoSpawnPt.heading

rightProjectPt = rightLaneSec.centerline.project(egoSpawnPt.position)
rightMotoSpawnPt = new OrientedPoint at rightProjectPt, facing egoSpawnPt.heading

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(5, 8)
param ADV_BRAKE = 1.0
param OPT_ADV_DECEL_TIME = Range(2, 4)

behavior RapidDecelBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) for globalParameters.OPT_ADV_DECEL_TIME seconds
    take SetBrakeAction(globalParameters.ADV_BRAKE)

adversary = new Car at leadCarSpawnPt,
    with heading leadCarSpawnPt.heading,
    with regionContainedIn None,
    with behavior RapidDecelBehavior()

param OPT_MOTO_SPEED = Range(5, 9)

behavior MotoForwardBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_MOTO_SPEED)

motorcycleAdv = new Motorcycle at rightMotoSpawnPt,
    with heading rightMotoSpawnPt.heading,
    with regionContainedIn None,
    with behavior MotoForwardBehavior()

param OPT_ADV3_SPEED = Range(6, 9)

behavior ForwardBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV3_SPEED)

adversary3 = new Car at leftCarSpawnPt,
    with heading leftCarSpawnPt.heading,
    with regionContainedIn None,
    with behavior ForwardBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        wait

require monitor TrafficLights()
require 10 <= (distance from ego to adversary) <= 20
require 3 <= (distance from ego to adversary3) <= 5
terminate when (ego intersects adversary)