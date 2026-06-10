description = "Ego vehicle turns left at an urban intersection, taking evasive action to avoid an obstacle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoInitLane.maneuvers))
propSpawnPt = new OrientedPoint in egoManeuver.connectingLane.centerline

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

prop = new Trash at propSpawnPt

TERM_DIST = 50

monitor TrafficLightControl():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightControl()
terminate when (distance from ego to egoSpawnPt) > TERM_DIST
