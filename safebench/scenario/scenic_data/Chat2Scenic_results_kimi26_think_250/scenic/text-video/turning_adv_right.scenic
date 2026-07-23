description = "Ego vehicle rear-ends a black sedan turning right from a side road at an intersection."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advManeuver.conflictingManeuvers))
egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline
advSpawnPt = new OrientedPoint in advManeuver.startLane.centerline

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=10, trajectory=egoTrajectory)
    interrupt when withinDistanceToAnyCars(self, 1.0):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)
param ADV_BRAKE = Range(0.5, 1.0)
SAFE_DIST = 20

behavior AdversaryBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyCars(self, SAFE_DIST):
		take SetBrakeAction(globalParameters.ADV_BRAKE)

adversary = new Car at advSpawnPt,
	with behavior AdversaryBehavior(advTrajectory)

EGO_INIT_DIST = [20, 25]
ADV_INIT_DIST = [15, 20]
TERM_DIST = 70

require EGO_INIT_DIST[0] <= (distance to intersection) <= EGO_INIT_DIST[1]
require ADV_INIT_DIST[0] <= (distance from adversary to intersection) <= ADV_INIT_DIST[1]
terminate when (distance to egoSpawnPt) > TERM_DIST