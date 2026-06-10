description = "Ego performs unprotected left turn, reacting to sudden braking of oncoming vehicle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*network.intersections)

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint in advInitLane.centerline
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

param OPT_EGO_SPEED = Range(3, 5)
param OPT_EGO_YIELD_DIST = Range(15, 20)
param OPT_EGO_ACCEL_SPEED = Range(6, 8)
OPT_EGO_DECISION_DEGREE = 25 deg

behavior EgoBehavior():
    initialDir = self.heading
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyCars(self, globalParameters.OPT_EGO_YIELD_DIST):
        if abs(self.heading - initialDir) < OPT_EGO_DECISION_DEGREE:
            take SetBrakeAction(1.0)
            take SetThrottleAction(0.0)
        else:
            do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_ACCEL_SPEED)
            abort
    terminate

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(6, 10)
param OPT_ADV_TRIGGER_DIST = Range(5, 10)

behavior WaitBehavior():
    while True:
        wait

behavior AdvBehavior():
    do FollowTrajectoryBehavior(trajectory=advTrajectory, target_speed=globalParameters.OPT_ADV_SPEED) until (distance from self to intersection <= globalParameters.OPT_ADV_TRIGGER_DIST)
    take SetThrottleAction(0.0)
    take SetBrakeAction(1.0)
    do WaitBehavior()

advAgent = new Car at advSpawnPt,
    with blueprint MODEL,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

EGO_INIT_DIST = [20, 25]
ADV_INIT_DIST = [15, 20]
TERM_DIST = 70

require EGO_INIT_DIST[0] <= (distance from egoSpawnPt to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from advSpawnPt to intersection) <= ADV_INIT_DIST[1]

monitor TrafficLightControl:
    freezeTrafficLights()
    setClosestTrafficLightStatus(ego, 'green')
    setClosestTrafficLightStatus(advAgent, 'green')
    while True:
        wait

require monitor TrafficLightControl()

terminate when (distance from ego to egoSpawnPt) > TERM_DIST