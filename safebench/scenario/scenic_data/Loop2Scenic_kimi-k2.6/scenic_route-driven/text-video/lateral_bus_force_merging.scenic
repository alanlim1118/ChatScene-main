description = "Using map ../../maps/Town04.xodr with carla map Town04 and weather ClearSunset"
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoLaneSec = network.laneSectionAt(egoSpawnPt)

advLaneSec = egoLaneSec._laneToRight
advLanePt = advLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advLanePt for Range(-30, -10)

egoInitLane = egoLaneSec.lane
advInitLane = advLaneSec.lane

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero'

param OPT_ADV_SPEED = Range(11, 13)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED, laneToFollow=advInitLane)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.OPT_ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED, laneToFollow=egoInitLane)

adversary = new Truck at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

streetSignSpawnPt = new OrientedPoint ahead of egoSpawnPt by Range(50, 80)
streetSign = new Prop at streetSignSpawnPt,
    with blueprint 'static.prop.streetsign'

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
require (distance from advSpawnPt to intersection) >= 10
terminate when (distance from ego to adversary) > 80
terminate after 15 seconds