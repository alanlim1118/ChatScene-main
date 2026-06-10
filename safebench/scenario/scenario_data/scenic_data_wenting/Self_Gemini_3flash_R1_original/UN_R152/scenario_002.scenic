description = "Subject vehicle follows a lead vehicle which then turns at an intersection, while the subject vehicle continues straight."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: any(any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers) and any(m.type is ManeuverType.LEFT_TURN or m.type is ManeuverType.RIGHT_TURN for m in l.maneuvers) for l in i.incomingLanes), network.intersections))

egoInitLane = Uniform(*filter(lambda l: any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers) and any(m.type is ManeuverType.LEFT_TURN or m.type is ManeuverType.RIGHT_TURN for m in l.maneuvers), intersection.incomingLanes))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN or m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))

advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint on advInitLane.centerline
egoSpawnPt = new OrientedPoint behind advSpawnPt by Range(10, 15)

egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(7, 10)
param SAFETY_DISTANCE = 10

behavior EgoBehavior(trajectory, speed, safety_dist):
    try:
        do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
    interrupt when withinDistanceToObjsInLane(self, safety_dist):
        take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory, globalParameters.EGO_SPEED, globalParameters.SAFETY_DISTANCE)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

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
require 15 <= (distance from advSpawnPt to intersection) <= 25
terminate when (distance from ego to adversary) > 30