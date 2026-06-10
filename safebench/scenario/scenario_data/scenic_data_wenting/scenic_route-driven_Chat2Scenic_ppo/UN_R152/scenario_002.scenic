description = "Subject vehicle follows a lead vehicle which then turns at an intersection, while the subject vehicle continues straight."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw
egoInitLane = network.laneAt(egoSpawnPt.position)

intersection = Uniform(*filter(lambda i: any(any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers) and any(m.type is ManeuverType.LEFT_TURN or m.type is ManeuverType.RIGHT_TURN for m in l.maneuvers) for l in i.incomingLanes), network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN or m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))

advInitLane = advManeuver.startLane
require advInitLane == egoInitLane

advSpawnPt = new OrientedPoint following advInitLane.orientation from egoSpawnPt for Range(10, 15)

advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectory)

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 50):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adversary, 50):
            setClosestTrafficLightStatus(adversary, "green")
        wait

require monitor TrafficLights()
require 10 <= (distance from egoSpawnPt to advSpawnPt) <= 15
require 15 <= (distance from advSpawnPt to intersection) <= 25
terminate when (distance from ego to adversary) > 30
