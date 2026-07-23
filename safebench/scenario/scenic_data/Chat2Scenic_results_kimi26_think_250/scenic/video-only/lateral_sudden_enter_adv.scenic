description = "Ego vehicle collides with a silver sedan that pulls from the right shoulder and cuts across the lane to turn left on a rural highway."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline
intSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(20, 40)
advSpawnPt = new OrientedPoint right of intSpawnPt by Range(3, 5), facing intSpawnPt.heading

param OPT_EGO_SPEED = Range(15, 20)
param OPT_BRAKE_THRESHOLD = Range(10, 15)

behavior EgoBehavior(speed, threshold):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToAnyCars(self, threshold)
    take SetBrakeAction(1)
    take SetSteerAction(-0.8)
    wait for 1.5 seconds
    take SetSteerAction(0)
    take SetBrakeAction(1)
    wait

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(OPT_EGO_SPEED, OPT_BRAKE_THRESHOLD)

behavior AdversaryBehavior():
    take SetHandBrakeAction(False)
    take SetThrottleAction(0.7)
    take SetSteerAction(-0.8)
    wait for 2.0 seconds
    take SetSteerAction(0)
    take SetBrakeAction(1)
    wait

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

require 20 <= (distance from ego to adversary) <= 45
terminate when ((distance from ego to advSpawnPt) < 15) and ((distance from ego to egoSpawnPt) > 10)