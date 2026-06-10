description = "Subject vehicle navigates a small radius curved road with outer guard pipes, avoiding a stationary target (vehicle, pedestrian, or bicycle) positioned just outside the pipes on the lane's center extension."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*network.intersections)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN or m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.connectingLane

egoSpawnPt = new OrientedPoint on egoInitLane.centerline

param ADV_TARGET_DIST = Range(20, 30)
advSpawnPt = new OrientedPoint ahead of egoSpawnPt by globalParameters.ADV_TARGET_DIST


pipeSpawnPt = new OrientedPoint ahead of egoSpawnPt by (globalParameters.ADV_TARGET_DIST - 3.0)

egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]

param EGO_SPEED = Range(6, 9)
param SAFETY_DISTANCE = 10

behavior EgoBehavior(trajectory, speed, safety_distance):
    try:
        do FollowTrajectoryBehavior(target_speed=speed, trajectory=trajectory)
    interrupt when withinDistanceToAnyObjs(self, safety_distance):
        take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory, globalParameters.EGO_SPEED, globalParameters.SAFETY_DISTANCE)

adversary = new Car at advSpawnPt,
    with regionContainedIn None


monitor TrafficLights():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLights()
require 10 <= (distance from egoSpawnPt to intersection) <= 30

terminate when (distance from ego to egoSpawnPt) > (globalParameters.ADV_TARGET_DIST + 20)
terminate after 40 seconds