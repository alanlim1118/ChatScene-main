description = "Ego vehicle encounters a stationary heavy truck in a curve and brakes to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on egoLane.centerline
truckSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for Range(50, 70)

param OPT_EGO_SPEED = Range(10, 15)
param OPT_BRAKE_DIST = Range(15, 25)

behavior EgoBehavior(speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DIST)

truck = new Truck at truckSpawnPt,
    with heading truckSpawnPt.heading,
    with regionContainedIn None

require 50 <= (distance from egoSpawnPt to truckSpawnPt) <= 70
terminate after 30 seconds