description = "Ego vehicle approaches a stationary pedestrian at high velocity, performing autonomous braking to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'walker.pedestrian.0001'
param weather = 'ClearNoon'

param OPT_PED_DISTANCE = Range(40, 60)

egoLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoLane.centerline

pedSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.OPT_PED_DISTANCE

param OPT_EGO_SPEED = Range(15, 20)
param OPT_BRAKE_THRESHOLD = Range(20, 25)
param EGO_MODEL = 'vehicle.tesla.model3'

behavior EgoBehavior(speed, threshold):
    do FollowLaneBehavior(target_speed=speed)


ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint globalParameters.EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_THRESHOLD)

param OPT_PED_SPEED = 0

behavior AdversaryBehavior(speed):
    take SetWalkingSpeedAction(speed)
    while True:
        wait

adversary = new Pedestrian at pedSpawnPt,
    with blueprint MODEL,
    with regionContainedIn None,
    with behavior AdversaryBehavior(globalParameters.OPT_PED_SPEED)

require ego can see adversary

terminate when (distance from ego to egoSpawnPt) > (globalParameters.OPT_PED_DISTANCE + 10)
terminate after 30 seconds