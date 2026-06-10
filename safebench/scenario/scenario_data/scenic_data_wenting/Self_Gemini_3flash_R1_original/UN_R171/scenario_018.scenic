description = "Ego vehicle follows a lead car that swerves to avoid a stationary obstacle."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param EGO_TO_LEAD_DIST = Range(10, 15)
param LEAD_TO_OBSTACLE_DIST = Range(20, 30)

egoLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoLane.centerline

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.EGO_TO_LEAD_DIST

obstacleSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.LEAD_TO_OBSTACLE_DIST

param OPT_EGO_SPEED = Range(10, 15)
param OPT_BRAKE_DIST = 12

behavior EgoBehavior(target_speed, brake_dist):
    do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DIST)

param ADV_SPEED = Range(10, 15)
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
    with behavior LeadCarBehavior(globalParameters.ADV_SPEED, globalParameters.ADV_SWERVE_DIST, egoLane)

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