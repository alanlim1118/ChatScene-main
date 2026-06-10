description = "Ego performs unprotected left turn while adversary makes a last-second right turn."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_EGO_DIST = Range(5, 10)
param OPT_ADV_DIST = Range(15, 25)

intersection = Uniform(*network.intersections)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoSpawnPt = new OrientedPoint following egoInitLane.orientation from egoInitLane.centerline.end for -globalParameters.OPT_EGO_DIST
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advSpawnPt = new OrientedPoint following advInitLane.orientation from advInitLane.centerline.end for -globalParameters.OPT_ADV_DIST
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

egoDir = egoSpawnPt.heading
advDir = advSpawnPt.heading

param OPT_EGO_SPEED = Range(4, 6)
param OPT_EGO_YIELD_DIST = Range(7, 10)

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyCars(self, globalParameters.OPT_EGO_YIELD_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_ADV_SPEED = Range(3, 6)
param OPT_ADV_LATETURN_DIST = Range(1, 3)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED, laneToFollow=advInitLane) until (distance from self to advManeuver.connectingLane.centerline < globalParameters.OPT_ADV_LATETURN_DIST)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED, laneToFollow=advInitLane) until (distance from self to advManeuver.connectingLane.centerline > globalParameters.OPT_ADV_LATETURN_DIST)
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adv = new Car at advSpawnPt,
    with blueprint MODEL,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

param HEADING_TOL = 20 deg

monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        if withinDistanceToTrafficLight(adv, 100):
            setClosestTrafficLightStatus(adv, "green")
        wait

require monitor TrafficLights()
require (relative heading of egoDir from advDir) > (180 deg - globalParameters.HEADING_TOL) or (relative heading of egoDir from advDir) < (-180 deg + globalParameters.HEADING_TOL)
require 5 <= (distance from egoSpawnPt to intersection) <= 15
require 10 <= (distance from advSpawnPt to intersection) <= 30

terminate when (distance from ego to egoSpawnPt) > 50
terminate after 40 seconds