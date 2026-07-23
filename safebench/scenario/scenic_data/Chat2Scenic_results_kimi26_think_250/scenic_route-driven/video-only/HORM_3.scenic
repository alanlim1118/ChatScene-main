description = "Ego vehicle merges from an on-ramp onto a multi-lane highway, positioning ahead of adjacent lane vehicles."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLane = network.laneAt(egoSpawnPt.position)
egoSec = network.laneSectionAt(egoSpawnPt)
leftSec = egoSec._laneToLeft
leftLane = leftSec.lane
leadCarSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for Range(15, 30)
leftCarSpawnPt = new OrientedPoint in leftLane.centerline
leftTruckSpawnPt = new OrientedPoint following leftLane.orientation from leftCarSpawnPt for Range(10, 20)

ego = new Car at egoSpawnPt,
    with blueprint MODEL

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=10)

adversary = new Car at leadCarSpawnPt,
    with heading leadCarSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

param OPT_LEFT_CAR_SPEED = Range(8, 12)

behavior LeftAdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEFT_CAR_SPEED)

left_adv = new Car at leftCarSpawnPt,
    with heading leftCarSpawnPt.heading,
    with regionContainedIn None,
    with behavior LeftAdvBehavior()

param OPT_LEFT_TRUCK_SPEED = Range(8, 12)

behavior TruckAdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_LEFT_TRUCK_SPEED)

truck = new Truck at leftTruckSpawnPt,
    with heading leftTruckSpawnPt.heading,
    with regionContainedIn None,
    with behavior TruckAdvBehavior()