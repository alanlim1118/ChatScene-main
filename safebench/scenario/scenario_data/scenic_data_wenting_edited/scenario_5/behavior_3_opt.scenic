description = "Ego vehicle avoids collision with an adversarial agent running a red light and making an abrupt left turn."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoSpawnPt = new OrientedPoint on egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint on advInitLane.centerline
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param EGO_SPEED = Range(7, 10)
param BRAKE_THRESHOLD = Range(10, 15)

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)
    interrupt when withinDistanceToAnyCars(self, globalParameters.BRAKE_THRESHOLD):
        take SetBrakeAction(1.0)
        take SetThrottleAction(0.0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(7, 10)
param OPT_ADV_DISTANCE = Range(40, 50)

behavior WaitBehavior():
    while True:
        wait

behavior AdvBehavior():
    do WaitBehavior() until (distance from self to ego) < globalParameters.OPT_ADV_DISTANCE
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

advAgent = new Car at advSpawnPt,
    with blueprint MODEL,
    with regionContainedIn None,
    with behavior AdvBehavior()

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(advAgent, 100):
            setClosestTrafficLightStatus(advAgent, "red")
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()

terminate when (ego in egoManeuver.endLane) and (distance from ego to egoSpawnPt > 30)