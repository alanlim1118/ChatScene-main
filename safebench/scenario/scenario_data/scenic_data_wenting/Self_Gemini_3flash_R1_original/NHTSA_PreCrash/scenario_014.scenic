description = "Ego vehicle performs a lane change in an urban area at high speed, then encroaches into another vehicle traveling in the same direction."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

advLaneSec = egoLaneSec._laneToLeft
advSpawnPt = new OrientedPoint at advLaneSec.centerline.project(egoSpawnPt.position), facing egoSpawnPt.heading

egoInitLane = egoLaneSec.lane
advInitLane = advLaneSec.lane

param EGO_SPEED = Range(20, 25)
param PRE_LANE_CHANGE_TIME = Range(1, 3)

behavior EgoBehavior(speed, target_lane):
    do FollowLaneBehavior(target_speed=speed) for globalParameters.PRE_LANE_CHANGE_TIME seconds
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.EGO_SPEED, advLaneSec)

param ADV_SPEED = Range(15, 20)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 100):
            setClosestTrafficLightStatus(adversary, "green")
        wait

require monitor TrafficLights()
terminate after 15 seconds