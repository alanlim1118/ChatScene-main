description = "Ego vehicle collides with a grey van that abruptly swerves into its lane to avoid a parked motorcycle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_CAR_DIST = Range(15, 25)
param OPT_CAR_MOTO_DIST = Range(5, 10)
param OPT_CURB_OFFSET = Range(2, 4)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

carForwardPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_EGO_CAR_DIST
carSpawnPt = new OrientedPoint right of carForwardPt by globalParameters.OPT_CURB_OFFSET

motoForwardPt = new OrientedPoint following roadDirection from carForwardPt for globalParameters.OPT_CAR_MOTO_DIST
motoSpawnPt = new OrientedPoint right of motoForwardPt by globalParameters.OPT_CURB_OFFSET

ego = new Car at egoSpawnPt,
    with blueprint MODEL

param OPT_ADV_SPEED = Range(5, 9)

behavior AdvBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.OPT_ADV_SPEED)
    while True:
        take SetBrakeAction(1)

adversary = new Car at carSpawnPt,
    with behavior AdvBehavior()

behavior StationaryBehavior():
    while True:
        wait

motorcycle = new Motorcycle at motoSpawnPt,
    with heading motoSpawnPt.heading,
    with regionContainedIn None,
    with behavior StationaryBehavior()

require 15 <= (distance from ego to adversary) <= 26
require 5 <= (distance from adversary to motorcycle) <= 10
terminate when ego intersects adversary