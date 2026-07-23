description = "Ego vehicle traveling at 50 km/h strikes a pedestrian crossing perpendicularly from the left."
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
advRefPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(20, 40)
advSpawnPt = new OrientedPoint left of advRefPt by Range(3, 5), facing toward advRefPt

param EGO_SPEED = Range(13, 14)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_PED_SPEED = Range(3, 5)

behavior PedestrianBehavior():
	do CrossingBehavior(ego, min_speed=globalParameters.ADV_PED_SPEED, threshold=25)

adversarial = new Pedestrian at advSpawnPt,
	with regionContainedIn None,
	with behavior PedestrianBehavior()

terminate when ego intersects adversarial