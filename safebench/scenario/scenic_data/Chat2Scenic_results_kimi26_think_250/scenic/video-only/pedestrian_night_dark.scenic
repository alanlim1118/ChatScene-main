description = "Ego vehicle travels on a dark highway at night when a pedestrian emerges from the left, illuminated only by headlights, forcing emergency braking and swerving."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way and not i.isSignalized, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectoryLine = egoInitLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline

egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline
IntSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(20, 35)
pedSpawnPt = new OrientedPoint left of IntSpawnPt by Range(3, 6)

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = Range(0.5, 1.0)
param EGO_SWERVE = Range(0.6, 1.0)
SAFE_DIST = 15

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, SAFE_DIST):
        take RegulatedControlAction(throttle=-globalParameters.EGO_BRAKE, steer=globalParameters.EGO_SWERVE, past_steer=0.0, max_brake=1.0, max_steer=1.0)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

behavior PedestrianBehavior():
    do CrossingBehavior(ego, 1, 25)

ped = new Pedestrian at pedSpawnPt,
    facing toward IntSpawnPt,
    with regionContainedIn None,
    with behavior PedestrianBehavior()