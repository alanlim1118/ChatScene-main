description = "Ego vehicle maneuvers to avoid a parked car opening its door into the lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoSection = network.laneSectionAt(egoSpawnPt)

require egoSection.isForward

egoEdgePos = egoSection.rightEdge.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following egoSection.orientation from egoEdgePos for Range(30, 50)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_TRIGGER_DIST = Range(15, 20)

behavior AdversarialBehavior(trigger_dist):
    wait until (distance from self to ego) < trigger_dist
    while True:
        take SetHandBrakeAction(True)

adversarial = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversarialBehavior(globalParameters.OPT_ADV_TRIGGER_DIST)

require 30 <= (distance from egoSpawnPt to advSpawnPt) <= 50
terminate when (distance from ego to advSpawnPt) > 70
terminate after 50 seconds
