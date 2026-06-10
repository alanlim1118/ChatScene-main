description = "Ego vehicle approaches intersection; adversarial car on left accelerates, enters, and stops suddenly."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

validManeuvers = []
for inter in network.intersections:
    if inter.is4Way:
        for m in inter.maneuvers:
            if m.type is ManeuverType.STRAIGHT:
                for sec in m.startLane.sections:
                    if sec._laneToLeft is not None:
                        validManeuvers.append(m)
                        break

egoManeuver = Uniform(*validManeuvers)
egoInitLane = egoManeuver.startLane

egoSection = Uniform(*filter(lambda s: s._laneToLeft is not None, egoInitLane.sections))
advSection = egoSection._laneToLeft
advInitLane = advSection.lane

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advInitLane.maneuvers))

egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]

egoSpawnPt = new OrientedPoint on egoInitLane.centerline
advSpawnPt = new OrientedPoint on advInitLane.centerline

param EGO_SPEED = 10

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param OPT_ADV_SPEED = globalParameters.EGO_SPEED * 0.5
param OPT_ADV_ACCEL_SPEED = globalParameters.EGO_SPEED * 1.8
param OPT_ACCEL_DURATION = 1.5
param OPT_ADV_TRIGGER_DISTANCE = 10

behavior AdvBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=trajectory) until (distance from self to trajectory[1] < globalParameters.OPT_ADV_TRIGGER_DISTANCE)
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_ACCEL_SPEED, trajectory=trajectory) for globalParameters.OPT_ACCEL_DURATION seconds
    take SetThrottleAction(0)
    take SetBrakeAction(1)

advCar = new Car at advSpawnPt,
    with blueprint MODEL,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior(advTrajectory)

require 30 <= (distance from egoSpawnPt to intersection) <= 40
require 30 <= (distance from advSpawnPt to intersection) <= 40

monitor TrafficLightManager:
    freezeTrafficLights()
    setClosestTrafficLightStatus(ego, 'green')
    setClosestTrafficLightStatus(advCar, 'green')
    wait until False

require monitor TrafficLightManager()

terminate when (distance from ego to egoSpawnPt) > 70
terminate after 40 seconds