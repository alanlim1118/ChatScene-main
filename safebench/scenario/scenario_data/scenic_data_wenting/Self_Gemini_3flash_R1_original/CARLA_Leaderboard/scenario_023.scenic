description = "Ego vehicle yields to a parked vehicle exiting a parallel parking bay."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithParking = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec._laneToRight is not None and sec._laneToRight._laneToRight is None:
            laneSecsWithParking.append(sec)

selectedEgoSec = Uniform(*laneSecsWithParking)
egoSpawnPt = new OrientedPoint in selectedEgoSec.centerline

selectedAdvSec = selectedEgoSec._laneToRight
basePt = selectedAdvSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from basePt for Range(10, 20)

param OPT_EGO_SPEED = Range(7, 10)
param OPT_EGO_YIELD_DIST = Range(12, 18)

behavior EgoBehavior(target_speed, yield_dist):
    do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_EGO_YIELD_DIST)

param OPT_ADV_SPEED = Range(5, 7)
param OPT_ADV_TRIGGER_DIST = Range(12, 18)

behavior ParkedCarBehavior(target_speed, trigger_dist, target_lane):
    wait until (distance from self to ego) < trigger_dist
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

adversarial = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior ParkedCarBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_TRIGGER_DIST, selectedEgoSec)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversarial, 100):
            setClosestTrafficLightStatus(adversarial, "green")
        wait

require monitor TrafficLights()
require 10 <= (distance from egoSpawnPt to advSpawnPt) <= 20
terminate when (distance from ego to egoSpawnPt) > 50