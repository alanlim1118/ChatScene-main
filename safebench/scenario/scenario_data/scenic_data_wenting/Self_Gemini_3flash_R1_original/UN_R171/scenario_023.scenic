description = "Ego vehicle encounters a sudden lane change from an adjacent heavy-duty truck."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

valid_pairs = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward:
            if sec._laneToLeft is not None and sec._laneToLeft.isForward:
                valid_pairs.append((sec, sec._laneToLeft))
            if sec._laneToRight is not None and sec._laneToRight.isForward:
                valid_pairs.append((sec, sec._laneToRight))

selected_pair = Uniform(*valid_pairs)
egoSection = selected_pair[0]
adjSection = selected_pair[1]

egoSpawnPt = new OrientedPoint in egoSection.centerline
truckBasePt = adjSection.centerline.project(egoSpawnPt.position)
truckSpawnPt = new OrientedPoint following adjSection.orientation from truckBasePt for Range(10, 20)

param EGO_SPEED = Range(10, 15)

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED)

param OPT_TRUCK_SPEED = globalParameters.EGO_SPEED - 2
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