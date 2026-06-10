description = "Ego vehicle turns right at an intersection as an adversarial motorcyclist abruptly crosses from the opposite sidewalk and stops in the center."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoTrajectoryLine = egoInitLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

conflictingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advLane = conflictingManeuver.startLane
intersectionCenter = intersection.polygon.centroid
tempPt = new OrientedPoint at advLane.rightEdge.points[-1], facing advLane.orientation[advLane.rightEdge.points[-1]]
motorcycleSpawnPt = new OrientedPoint right of tempPt by 3, facing toward intersectionCenter
motorcycleTargetPt = new OrientedPoint at intersectionCenter

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param OPT_ADV_SPEED = Range(3, 6)
param OPT_ADV_DISTANCE = Range(20, 30)
OPT_STOP_DISTANCE = 1.0

behavior MotorcycleBehavior(actor_ref, speed, threshold, stop_ref, stop_dist):
    do CrossingBehavior(actor_ref, speed, threshold) until (distance from self to stop_ref) < stop_dist
    take SetBrakeAction(1)
    take SetThrottleAction(0)

AdvAgent = new Motorcycle at motorcycleSpawnPt,
    with regionContainedIn None,
    with behavior MotorcycleBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE, motorcycleTargetPt, OPT_STOP_DISTANCE)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 50):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 10 <= (distance from egoSpawnPt to intersection) <= 25
require 1 <= (distance from motorcycleSpawnPt to intersection) <= 10

terminate when (distance from ego to egoSpawnPt) > 50
terminate after 30 seconds