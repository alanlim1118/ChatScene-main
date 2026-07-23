description = "Ego vehicle reacts to a suddenly stopping lead vehicle ahead, resulting in a minor collision during an evasive maneuver near an intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

intersection = Uniform(*network.intersections)
egoInitLane = Uniform(*intersection.incomingLanes)
advInitLane = egoInitLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(10, 20)
suvSpawnPt = new OrientedPoint right of advSpawnPt by Range(4, 8), facing advSpawnPt.heading

param OPT_EGO_SPEED = Range(5, 8)
param OPT_EVASIVE_DIST = Range(8, 12)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_EVASIVE_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        take SetSteerAction(1)
    terminate

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(6, 9)
param ADV_BRAKE = Range(0.8, 1.0)
param ADV_STEER = Range(0.6, 1.0)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for Range(3, 5) seconds
    while True:
        take SetBrakeAction(globalParameters.ADV_BRAKE)
        take SetSteerAction(globalParameters.ADV_STEER)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()