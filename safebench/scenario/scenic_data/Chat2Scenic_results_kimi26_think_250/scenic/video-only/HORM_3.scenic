description = "Ego vehicle merges from an on-ramp onto a multi-lane highway, positioning ahead of adjacent lane vehicles."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoLane = Uniform(*filter(lambda l: len(l.sections) > 0 and all(s._laneToLeft is not None for s in l.sections), network.lanes))
egoSec = Uniform(*egoLane.sections)
leftSec = egoSec._laneToLeft
leftLane = leftSec.lane
egoSpawnPt = new OrientedPoint in egoLane.centerline
leadCarSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for Range(15, 30)
leftCarSpawnPt = new OrientedPoint in leftLane.centerline
leftTruckSpawnPt = new OrientedPoint following leftLane.orientation from leftCarSpawnPt for Range(10, 20)

param EGO_SPEED = Range(8, 12)
param EGO_MERGE_DIST = Range(10, 20)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until (distance from self to egoSpawnPt) > globalParameters.EGO_MERGE_DIST
    do LaneChangeBehavior(laneSectionToSwitch=leftSec, target_speed=globalParameters.EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=10)

adversary = new Car at leadCarSpawnPt,
    with heading leadCarSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

param LEFT_CAR_SPEED = Range(8, 12)

behavior LeftAdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEFT_CAR_SPEED)

left_adv = new Car at leftCarSpawnPt,
    with heading leftCarSpawnPt.heading,
    with regionContainedIn None,
    with behavior LeftAdvBehavior()

param LEFT_TRUCK_SPEED = Range(8, 12)

behavior TruckAdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.LEFT_TRUCK_SPEED)

truck = new Truck at leftTruckSpawnPt,
    with heading leftTruckSpawnPt.heading,
    with regionContainedIn None,
    with behavior TruckAdvBehavior()