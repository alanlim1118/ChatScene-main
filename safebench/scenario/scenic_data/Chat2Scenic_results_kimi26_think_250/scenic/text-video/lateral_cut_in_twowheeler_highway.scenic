description = "Ego vehicle on a wet elevated highway rear-ends a fallen motorcycle and its two riders after they lose control and fall into the lane."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetNoon'

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on initLane.centerline
advSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(15, 40)

param EGO_SPEED = Range(12, 18)
param EGO_BRAKE_DIST = Range(8, 12)

behavior EgoBehavior(target_speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, brake_dist):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, globalParameters.EGO_BRAKE_DIST)

behavior MotorcycleBehavior():
    do FollowLaneBehavior(target_speed=15) for 3 seconds
    take SetSteerAction(1.0)
    while True:
        take SetBrakeAction(1.0)

advMotorcycle = new Motorcycle at advSpawnPt,
    facing advSpawnPt.heading,
    with behavior MotorcycleBehavior()

require not (ego intersects advMotorcycle)
terminate when ego intersects advMotorcycle