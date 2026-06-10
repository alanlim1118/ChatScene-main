description = "Ego vehicle follows a lead car that swerves to avoid a stationary obstacle."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_TO_LEAD_DIST = Range(10, 15)
param OPT_LEAD_TO_OBSTACLE_DIST = Range(20, 30)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoLane = network.laneAt(egoSpawnPt.position)

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_EGO_TO_LEAD_DIST

obstacleSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.OPT_LEAD_TO_OBSTACLE_DIST

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(10, 15)
param ADV_SWERVE_DIST = 15


behavior LeadCarBehavior(speed, swerve_dist, lane_obj):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, swerve_dist)
    if lane_obj.sections[0]._laneToLeft:
        do LaneChangeBehavior(laneSectionToSwitch=lane_obj.sections[0]._laneToLeft, target_speed=speed)
    elif lane_obj.sections[0]._laneToRight:
        do LaneChangeBehavior(laneSectionToSwitch=lane_obj.sections[0]._laneToRight, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

leadCar = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior LeadCarBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.ADV_SWERVE_DIST, egoLane)

behavior WaitBehavior():
    while True:
        wait

AdvAgent = new Motorcycle at obstacleSpawnPt,
    with behavior WaitBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(leadCar, 100):
            setClosestTrafficLightStatus(leadCar, "green")
        wait

require monitor TrafficLights()


terminate when (distance from ego to egoSpawnPt) > 80
