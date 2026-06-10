description = "Ego vehicle approaches an intersection, yielding to a crossing vehicle from the side to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))

egoInitLane = Uniform(*filter(lambda l: any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers), intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and any(c.type is ManeuverType.STRAIGHT for c in m.conflictingManeuvers), egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = Range(0.5, 1.0)
param BRAKE_DIST = Range(15, 20)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectory)

monitor TrafficLightControl():
    freezeTrafficLights()
    while True:
        setClosestTrafficLightStatus(ego, "green")
        setClosestTrafficLightStatus(adversary, "green")
        wait

require monitor TrafficLightControl()
require 30 <= (distance from egoSpawnPt to intersection) <= 40
terminate when (ego intersects adversary) or (distance from ego to egoSpawnPt > 60)