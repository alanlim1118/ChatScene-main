description = "Ego vehicle turns left at an intersection; adversarial pedestrian suddenly crosses road and stops in middle."
Town = globalParameters.town
EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
lanePts = globalParameters.lanePts
egoTrajectory = PolylineRegion(globalParameters.waypoints)
param map = localPath(f'../maps/{Town}.xodr')
param carla_map = Town
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = globalParameters.weather

egoInitLane = network.laneAt(lanePts[0])
egoConnectingLane = network.laneAt(lanePts[1])
egoEndLane = network.laneAt(lanePts[2])
egoManeuver = Uniform(*filter(
    lambda m: (m.type is ManeuverType.LEFT_TURN
               and m.connectingLane is egoConnectingLane
               and m.endLane is egoEndLane),
    egoInitLane.maneuvers))

oppManeuver = Uniform(*filter(
    lambda m: m.type is ManeuverType.STRAIGHT or m.type is ManeuverType.RIGHT_TURN,
    egoManeuver.reverseManeuvers))
oppInitLane = oppManeuver.startLane
pedRefPt = new OrientedPoint on oppInitLane.centerline
pedSpawnPt = new OrientedPoint right of pedRefPt by 5

intersection = egoManeuver.intersection

param OPT_PED_SPEED = Range(1.0, 2.0)
param OPT_PED_THRESHOLD = Range(15, 25)

behavior CrossAndStopBehavior(actor_reference, min_speed, threshold, stop_reference):
    do CrossingBehavior(actor_reference, min_speed, threshold) until (distance from self to stop_reference) <= 1.0
    take SetWalkingSpeedAction(0)

ego = new Car at EgoSpawnPt,
    facing yaw,
    with regionContainedIn None,
    with blueprint MODEL

ped = new Pedestrian at pedSpawnPt,
    facing 90 deg relative to pedRefPt.heading,
    with regionContainedIn None,
    with behavior CrossAndStopBehavior(ego, globalParameters.OPT_PED_SPEED, globalParameters.OPT_PED_THRESHOLD, pedRefPt)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 15 <= (distance from EgoSpawnPt to intersection) <= 35
require 10 <= (distance from pedRefPt to intersection) <= 30

terminate when (distance from ego to EgoSpawnPt) > 60
terminate after 40 seconds
