description = "Ego vehicle turning left at intersection; adversarial motorcyclist feigns crossing and brakes abruptly."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.yamaha.yzf'
param weather = 'ClearNoon'

param OPT_ADV_DIST = Range(6, 10)
param OPT_LATERAL_OFFSET = 3.5

intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

egoSpawnPt = new OrientedPoint behind egoInitLane.centerline.end by Range(20, 25)

tempPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for OPT_ADV_DIST
advSpawnPt = new OrientedPoint right of tempPt by OPT_LATERAL_OFFSET

advInitLane = network.laneSectionAt(advSpawnPt).lane
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param OPT_ADV_SPEED = Range(10, 15)
param OPT_TRIGGER_DIST = Range(12, 18)

behavior BrakeAtEdgeBehavior(speed, trajectory, stopPt):
    wait until (distance to ego) < globalParameters.OPT_TRIGGER_DIST
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory) until (distance from self to stopPt) < 2.0
    take SetBrakeAction(1.0)
    take SetThrottleAction(0.0)

advMotorcycle = new Motorcycle at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior BrakeAtEdgeBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory, advInitLane.centerline.end)

monitor TrafficLightControl():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, 'green')
        if withinDistanceToTrafficLight(advMotorcycle, 100):
            setClosestTrafficLightStatus(advMotorcycle, 'green')
        wait

require monitor TrafficLightControl()

terminate when (distance from ego to egoSpawnPt) > 80
terminate after 60 seconds