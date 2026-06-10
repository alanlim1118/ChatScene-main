description = "Lead vehicle aggressive lane change to reveal stationary object, testing ego sensor tracking."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_TO_LEAD_DIST = Range(15, 20)
param OPT_LEAD_TO_PROP_DIST = Range(10, 15)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLaneSec = network.laneSectionAt(egoSpawnPt)

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_EGO_TO_LEAD_DIST
propSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.OPT_LEAD_TO_PROP_DIST

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn egoLaneSec,
    with blueprint MODEL

param OPT_LEAD_SPEED = Range(15, 20)
param OPT_SWERVE_DIST = Range(10, 15)

behavior LeadBehavior(speed, swerve_dist, target_point):
    do FollowLaneBehavior(target_speed=speed) until (distance from self to target_point) < swerve_dist
    if self.laneSection._laneToLeft:
        targetLane = self.laneSection._laneToLeft
    else:
        targetLane = self.laneSection._laneToRight
    do LaneChangeBehavior(laneSectionToSwitch=targetLane, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

lead = new Car at leadSpawnPt,
    with blueprint MODEL,
    with regionContainedIn None,
    with behavior LeadBehavior(globalParameters.OPT_LEAD_SPEED, globalParameters.OPT_SWERVE_DIST, propSpawnPt)

prop = new Trash at propSpawnPt

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(lead, 100):
            setClosestTrafficLightStatus(lead, "green")
        wait

require monitor TrafficLights()
terminate when (distance from ego to egoSpawnPt) > 70
terminate after 30 seconds
