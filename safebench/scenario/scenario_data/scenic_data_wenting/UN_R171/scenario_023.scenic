description = "Ego vehicle encounters a sudden lane change from an adjacent heavy-duty truck."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoSection = network.laneSectionAt(egoSpawnPt)

adjSection = egoSection._laneToLeft if (egoSection._laneToLeft is not None and egoSection._laneToLeft.isForward) else egoSection._laneToRight
require adjSection is not None
require adjSection.isForward

truckBasePt = adjSection.centerline.project(egoSpawnPt.position)
truckSpawnPt = new OrientedPoint following adjSection.orientation from truckBasePt for Range(10, 20)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_TRUCK_SPEED = Range(8, 13)
param OPT_TRIGGER_DIST = Range(12, 18)

behavior TruckBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED) until (distance from self to ego < globalParameters.OPT_TRIGGER_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=egoSection, is_oppositeTraffic=False, target_speed=globalParameters.OPT_TRUCK_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED)

adversarialTruck = new Truck at truckSpawnPt,
    with behavior TruckBehavior()

param TERMINATE_DIST = 100

require 10 <= (distance from egoSpawnPt to truckSpawnPt) <= 25
terminate when (distance from ego to egoSpawnPt) > globalParameters.TERMINATE_DIST
