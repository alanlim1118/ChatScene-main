description = "No header settings provided"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_BLOCK_DIST = Range(20, 30)
param OPT_LEADING_DIST = Range(10, 20)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoLaneSec = network.laneSectionAt(egoSpawnPt)
adjLaneSec = egoLaneSec._laneToRight

LeadingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DIST

adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
AdvSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_ADV_BLOCK_DIST

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with rolename 'hero'

param OPT_ADV_REVERSE_DIST = Range(15, 25)
param OPT_ADV_CURVE_SPEED = Range(3, 5)

behavior AdvBehavior():
    take SetReverseAction(True)
    take SetThrottleAction(0.3)
    take SetSteerAction(-0.5)
    wait for globalParameters.OPT_ADV_REVERSE_DIST / globalParameters.OPT_ADV_CURVE_SPEED seconds
    take SetSteerAction(0.0)
    take SetThrottleAction(0.0)
    terminate

adversary = new Car at AdvSpawnPt,
    facing toward egoSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

require monitor TrafficLights()
require (distance from egoSpawnPt to intersection) >= 30
terminate when (distance from ego to adversary) < 5
terminate after 15 seconds