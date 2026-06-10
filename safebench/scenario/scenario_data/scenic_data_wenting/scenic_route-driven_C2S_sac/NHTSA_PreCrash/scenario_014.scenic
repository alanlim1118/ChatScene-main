description = "Ego vehicle performs a lane change in an urban area at high speed, then encroaches into another vehicle traveling in the same direction."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

advLaneSec = egoLaneSec._laneToLeft
require advLaneSec is not None
require advLaneSec.isForward
advSpawnPt = new OrientedPoint at advLaneSec.centerline.project(egoSpawnPt.position), facing egoSpawnPt.heading

egoInitLane = egoLaneSec.lane
advInitLane = advLaneSec.lane

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(15, 20)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

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
