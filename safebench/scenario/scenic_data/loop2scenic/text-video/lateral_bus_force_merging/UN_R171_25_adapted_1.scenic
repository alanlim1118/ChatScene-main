description = "Using map ../../maps/Town04.xodr with carla map Town04 and weather ClearSunset"
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearSunset'

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

advLaneSec = egoLaneSec._laneToRight
advLanePt = advLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from advLanePt for Range(-30, -10)

egoInitLane = egoLaneSec.lane
advInitLane = advLaneSec.lane

param EGO_SPEED = Range(9, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolename 'hero',
	with behavior EgoBehavior()

param ADV_SPEED = Range(11, 13)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, laneToFollow=advInitLane)
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED, laneToFollow=egoInitLane)

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