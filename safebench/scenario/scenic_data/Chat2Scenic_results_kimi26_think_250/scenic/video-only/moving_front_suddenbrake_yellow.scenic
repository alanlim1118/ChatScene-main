description = "Ego vehicle rear-ends a lead sedan after it brakes abruptly inside an intersection on a wide urban road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.startLane.road.forwardLanes is not None and len(m.startLane.road.forwardLanes.lanes) == 3 and len(m.startLane.adjacentLanes) == 2, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoLaneSec = Uniform(*egoInitLane.sections)
rightLaneSec = egoLaneSec._laneToRight
rightLane = rightLaneSec.lane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(10, 20)
npcSpawnPt = new OrientedPoint in rightLane.centerline
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

param OPT_EGO_SPEED = Range(8, 12)
param OPT_BRAKE_THRESHOLD = Range(8, 12)

behavior EgoBehavior(speed, threshold):
    do FollowTrajectoryBehavior(target_speed=speed, trajectory=egoTrajectory) until withinDistanceToObjsInLane(self, threshold)
    take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_THRESHOLD)

param OPT_ADV_SPEED = Range(8, 12)
param OPT_ADV_BRAKE_TIME = Range(2, 4)

behavior AdvBehavior():
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory) for globalParameters.OPT_ADV_BRAKE_TIME seconds
    while True:
        take SetBrakeAction(1)

adv = new Car at advSpawnPt,
    with regionContainedIn None,
    with behavior AdvBehavior()

param OPT_NPC_SPEED = Range(12, 16)

behavior NPCBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_NPC_SPEED)

npc = new NPCCar at npcSpawnPt,
    with regionContainedIn rightLaneSec,
    with behavior NPCBehavior()

param OPT_EGO_TERMINATE_DIST = Range(40, 60)

monitor TrafficSignal():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(adv, 100):
            setClosestTrafficLightStatus(adv, "red")
        wait

require monitor TrafficSignal()
require 10 <= (distance from ego to adv) <= 20
terminate when (ego intersects adv) or ((distance from egoSpawnPt to ego) > globalParameters.OPT_EGO_TERMINATE_DIST)