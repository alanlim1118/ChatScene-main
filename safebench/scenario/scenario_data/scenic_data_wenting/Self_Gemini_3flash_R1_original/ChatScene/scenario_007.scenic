description = "Ego vehicle turns left at an intersection; adversarial pedestrian suddenly crosses road and stops in middle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint on egoInitLane.centerline

oppManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT or m.type is ManeuverType.RIGHT_TURN, egoManeuver.reverseManeuvers))
oppInitLane = oppManeuver.startLane
pedRefPt = new OrientedPoint on oppInitLane.centerline
pedSpawnPt = new OrientedPoint right of pedRefPt by 5

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param OPT_PED_SPEED = Range(1.0, 2.0)
param OPT_PED_THRESHOLD = Range(15, 25)

behavior CrossAndStopBehavior(actor_reference, min_speed, threshold, stop_reference):
    do CrossingBehavior(actor_reference, min_speed, threshold) until (distance from self to stop_reference) <= 1.0
    take SetWalkingSpeedAction(0)

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
require 15 <= (distance from egoSpawnPt to intersection) <= 35
require 10 <= (distance from pedRefPt to intersection) <= 30

terminate when (distance from ego to egoSpawnPt) > 60
terminate after 40 seconds