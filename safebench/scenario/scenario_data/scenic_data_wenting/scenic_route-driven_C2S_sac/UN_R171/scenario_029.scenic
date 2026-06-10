description = "Ego vehicle approaches a stationary pedestrian at high velocity, performing autonomous braking to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'walker.pedestrian.0001'
param weather = 'ClearNoon'

param OPT_PED_DISTANCE = Range(40, 60)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLane = network.laneAt(egoSpawnPt.position)

pedSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.OPT_PED_DISTANCE

param EGO_MODEL = 'vehicle.tesla.model3'

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint globalParameters.EGO_MODEL

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
