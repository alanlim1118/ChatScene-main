description = "Ego vehicle encounters a slow moving hazard, requiring braking or maneuvering next to same-direction traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLaneSec = network.laneSectionAt(egoSpawnPt)

advLaneSec = egoLaneSec._laneToLeft if (egoLaneSec._laneToLeft is not None and egoLaneSec._laneToLeft.isForward == egoLaneSec.isForward) else egoLaneSec._laneToRight
require advLaneSec is not None
require advLaneSec.isForward == egoLaneSec.isForward

advSpawnPt = new OrientedPoint in advLaneSec.centerline
propSpawnPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for Range(30, 50)

require 5 <= (distance from egoSpawnPt to advSpawnPt) <= 20

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_PROP_SPEED = Range(2, 5)

behavior PropBehavior(speed):
	do FollowLaneBehavior(target_speed=speed)

advProp = new Car at propSpawnPt,
	with blueprint MODEL,
	with behavior PropBehavior(globalParameters.OPT_PROP_SPEED)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require (distance from egoSpawnPt to propSpawnPt) >= 35
terminate when (distance from ego to egoSpawnPt) > 100
