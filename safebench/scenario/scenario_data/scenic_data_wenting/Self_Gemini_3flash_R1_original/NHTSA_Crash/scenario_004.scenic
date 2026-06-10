description = "Vehicle A cuts a left turn too sharply, colliding with Vehicle B waiting at an intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: any(m.type is ManeuverType.LEFT_TURN for m in i.maneuvers), network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN and any(c.type is ManeuverType.STRAIGHT for c in m.conflictingManeuvers), intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint on egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint on advInitLane.centerline
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param OPT_EGO_SPEED = Range(5, 8)
param OPT_SHARP_STEER = Range(-0.7, -0.5)
param OPT_BRAKE_DIST = 1.0

behavior EgoBehavior(target_speed, steer_val):
    do FollowLaneBehavior(target_speed=target_speed) until self.lane == egoManeuver.connectingLane
    while self.lane == egoManeuver.connectingLane:
        take SetSteerAction(steer_val)
        take SetThrottleAction(0.3)
    do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        globalParameters.OPT_SHARP_STEER
    )

behavior WaitBehavior():
    while True:
        wait

advAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior WaitBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(advAgent, 100):
            setClosestTrafficLightStatus(advAgent, "red")
        wait

require monitor TrafficLights()
terminate after 15 seconds
terminate when (distance from ego to egoSpawnPt) > 50