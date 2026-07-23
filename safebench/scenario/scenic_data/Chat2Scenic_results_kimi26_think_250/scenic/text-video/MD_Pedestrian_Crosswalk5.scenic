description = "Ego vehicle brakes for a pedestrian crossing an intersection while an adversary vehicle enters from the left."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint in advInitLane.centerline
midPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for 30
pedSpawnPt = new OrientedPoint left of midPt by Range(4, 8)

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = Range(0.8, 1.0)
param EGO_INIT_DIST = Range(-30, -20)
SAFE_DIST = 20

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyPedestrians(self, SAFE_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car following roadDirection from egoSpawnPt for globalParameters.EGO_INIT_DIST,
    with blueprint MODEL,
    with behavior EgoBehavior()

PED_MIN_SPEED = 1.0
PED_THRESHOLD = 20

behavior PedestrianBehavior():
    do CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
    facing ego.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior(advTrajectory)

param EGO_INT_DIST = [20, 40]
param ADV_INT_DIST = [0, 20]

require EGO_INT_DIST[0] <= (distance to intersection) <= EGO_INT_DIST[1]
require ADV_INT_DIST[0] <= (distance from adversary to intersection) <= ADV_INT_DIST[1]
terminate when ego in egoManeuver.endLane